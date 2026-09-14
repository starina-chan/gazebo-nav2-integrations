import os

import numpy as np
import rclpy
from gazebo_msgs.srv import DeleteEntity, GetModelList, SpawnEntity
from nav_msgs.msg import OccupancyGrid
from rclpy.node import Node

from gazebo_nav2_integrations.map_to_gazebo import mesh, sdf, stl_writer
from gazebo_nav2_integrations.map_to_gazebo.gazebo_client import call_with_retry, poll_until


class MapToGazebo(Node):
    def __init__(self):
        super().__init__("map_to_gazebo")

        self.declare_parameter("map_topic", "/map")
        self.declare_parameter("occupied_thresh", 1)
        self.declare_parameter("box_height", 2.0)
        self.declare_parameter("export_dir", os.path.abspath("."))
        self.declare_parameter("model_name", "map")
        self.declare_parameter("spawn_in_gazebo", True)
        # Entities that must survive the wipe-before-respawn sweep (the
        # floor, and whatever the robot entity is named - defaults match
        # this workspace's turtlebot3 burger setup, override to match yours).
        self.declare_parameter("entities_to_keep", ["ground_plane", "burger"])
        # How long/often to poll /get_model_list to confirm a delete/spawn
        # actually landed before trusting it - a service call's success=True
        # only means Gazebo accepted the request, not that the model list
        # has caught up yet.
        self.declare_parameter("respawn_poll_interval_sec", 0.25)
        self.declare_parameter("respawn_poll_timeout_sec", 5.0)

        self.map_topic = self.get_parameter("map_topic").value
        self.threshold = self.get_parameter("occupied_thresh").value
        self.height = self.get_parameter("box_height").value
        self.export_dir = self.get_parameter("export_dir").value
        self.model_name = self.get_parameter("model_name").value
        self.spawn_in_gazebo = self.get_parameter("spawn_in_gazebo").value
        self.entities_to_keep = set(self.get_parameter("entities_to_keep").value)
        self.poll_interval = self.get_parameter("respawn_poll_interval_sec").value
        self.poll_timeout = self.get_parameter("respawn_poll_timeout_sec").value

        # Gazebo's renderer (Ogre) caches loaded mesh geometry keyed by its
        # file:// URI, and a respawned entity's runtime name must not collide
        # with the one being deleted this same cycle (gzclient's scenegraph
        # updates for the delete and the spawn can otherwise race/reorder) -
        # so both the mesh filename and the entity name carry an
        # ever-incrementing suffix, making every export a name/URI Ogre and
        # gzclient have never seen before.
        self._export_seq = 0
        self._pending_mesh_path = None
        self._last_mesh_path = None
        self._pending_entity_name = None
        self._delete_targets = []
        self._pending_deletes = 0

        if self.spawn_in_gazebo:
            self.delete_entity_client = self.create_client(DeleteEntity, "/delete_entity")
            self.spawn_entity_client = self.create_client(SpawnEntity, "/spawn_entity")
            self.get_model_list_client = self.create_client(GetModelList, "/get_model_list")

        self.map_sub = self.create_subscription(
            OccupancyGrid, self.map_topic, self.map_callback, 10
        )
        self.get_logger().info(f"map_to_gazebo running, export_dir={self.export_dir}")

    def map_callback(self, map_msg):
        info = map_msg.info
        map_array = np.array(map_msg.data).reshape(info.height, info.width)
        map_array[map_array < 0] = 0  # unknown (-1) -> unoccupied

        contours = mesh.get_occupied_regions(map_array, self.threshold)
        if not contours:
            self.get_logger().warning("no occupied regions in map, skipping export")
            return

        vertices, faces = mesh.contours_to_mesh(
            contours, info.resolution, info.origin.position.x, info.origin.position.y, self.height
        )

        os.makedirs(self.export_dir, exist_ok=True)
        self._export_seq += 1
        mesh_filename = f"map_{self._export_seq}.stl"
        mesh_path = os.path.join(self.export_dir, mesh_filename)
        stl_writer.write_binary_stl(mesh_path, vertices, faces)

        self._pending_mesh_path = mesh_path
        self._pending_entity_name = f"{self.model_name}_{self._export_seq}"
        self.get_logger().info(f"exported {mesh_path}")

        with open(os.path.join(self.export_dir, "model.config"), "w") as f:
            f.write(sdf.model_config_xml(self.model_name))
        with open(os.path.join(self.export_dir, "model.sdf"), "w") as f:
            f.write(sdf.model_sdf_xml(self._pending_entity_name, f"file://{mesh_path}"))
        with open(os.path.join(self.export_dir, f"{self.model_name}.world"), "w") as f:
            f.write(sdf.world_xml(self.export_dir))

        if self.spawn_in_gazebo:
            self._respawn_in_gazebo()

    def _respawn_in_gazebo(self):
        ready = (
            self.delete_entity_client.wait_for_service(timeout_sec=5.0)
            and self.spawn_entity_client.wait_for_service(timeout_sec=5.0)
            and self.get_model_list_client.wait_for_service(timeout_sec=5.0)
        )
        if not ready:
            self.get_logger().warning(
                "gazebo_ros entity services unavailable - skipping gazebo spawn for this export"
            )
            return
        future = self.get_model_list_client.call_async(GetModelList.Request())
        future.add_done_callback(self._on_model_list_for_delete)

    def _on_model_list_for_delete(self, future):
        try:
            response = future.result()
        except Exception as e:
            self.get_logger().warning(f"get_model_list failed ({e}), spawning without a wipe")
            self._spawn_exported_model()
            return

        targets = [n for n in response.model_names if n not in self.entities_to_keep]
        if not targets:
            self._spawn_exported_model()
            return

        self._delete_targets = targets
        self._pending_deletes = len(targets)
        for name in targets:
            future = self.delete_entity_client.call_async(DeleteEntity.Request(name=name))
            future.add_done_callback(lambda f, name=name: self._on_delete_done(f, name))

    def _on_delete_done(self, future, name):
        try:
            response = future.result()
            if response.success:
                self.get_logger().info(f"deleted existing gazebo entity '{name}'")
            else:
                self.get_logger().info(f"delete_entity('{name}'): {response.status_message}")
        except Exception as e:
            self.get_logger().warning(f"delete_entity('{name}') raised an exception: {e}")

        self._pending_deletes -= 1
        if self._pending_deletes == 0:
            poll_until(
                self,
                lambda: self.get_model_list_client.call_async(GetModelList.Request()),
                lambda response: not any(n in response.model_names for n in self._delete_targets),
                self.poll_interval,
                self.poll_timeout,
                self._spawn_exported_model,
            )

    def _spawn_exported_model(self):
        sdf_path = os.path.join(self.export_dir, "model.sdf")
        try:
            with open(sdf_path, "r") as f:
                sdf_xml = f.read()
        except OSError as e:
            self.get_logger().error(f"could not read {sdf_path} to spawn: {e}")
            return

        request = SpawnEntity.Request(
            name=self._pending_entity_name,
            xml=sdf_xml,
            robot_namespace="",
            reference_frame="world",
        )
        call_with_retry(self, self.spawn_entity_client, request, 5.0, self._on_spawn_done)

    def _on_spawn_done(self, future):
        try:
            response = future.result()
        except Exception as e:
            self.get_logger().error(f"spawn_entity raised an exception: {e}")
            return
        if not response.success:
            self.get_logger().error(
                f"failed to spawn '{self._pending_entity_name}': {response.status_message}"
            )
            return

        self.get_logger().info(
            f"spawn_entity acked '{self._pending_entity_name}' - confirming it's in the world"
        )
        poll_until(
            self,
            lambda: self.get_model_list_client.call_async(GetModelList.Request()),
            lambda response: self._pending_entity_name in response.model_names,
            self.poll_interval,
            self.poll_timeout,
            self._cleanup_previous_mesh,
        )

    def _cleanup_previous_mesh(self):
        """Delete the previous export's mesh file now that this export's
        entity is confirmed present, so map_N.stl files don't pile up in
        export_dir forever. Only runs after a confirmed spawn, so a failed
        respawn just leaves both files - harmless, self-heals next cycle.
        """
        old_path, self._last_mesh_path = self._last_mesh_path, self._pending_mesh_path
        if old_path and old_path != self._pending_mesh_path:
            try:
                os.remove(old_path)
            except OSError as e:
                self.get_logger().warning(f"could not remove stale mesh {old_path}: {e}")


def main(args=None):
    rclpy.init(args=args)
    node = MapToGazebo()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
