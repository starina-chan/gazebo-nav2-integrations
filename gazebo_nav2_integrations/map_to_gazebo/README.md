# map_to_gazebo

Subscribes to an `OccupancyGrid` topic, converts occupied cells into a mesh
(one box per boundary pixel of each occupied region, built as a single
vectorized numpy operation - no per-point mesh objects, no `trimesh`
dependency), exports it as a binary STL, writes `model.config`/`model.sdf`/a
`.world` file next to it, then wipes every existing Gazebo entity except
`entities_to_keep` and spawns the new model - confirming each step actually
landed by polling `/get_model_list` (a service call's `success=True` only
means Gazebo accepted the request, not that the world state has caught up).

## Usage

```
ros2 launch gazebo_nav2_integrations map_to_gazebo.launch.py export_dir:=/path/to/export
```

## Parameters (`config/map_to_gazebo.yaml`)

| Param | Default | Meaning |
|---|---|---|
| `map_topic` | `/map` | OccupancyGrid topic to convert |
| `occupied_thresh` | `1` | Minimum cell value (0-100) considered occupied |
| `box_height` | `2.0` | Height (m) of the extruded boxes |
| `export_dir` | Current working directory | Where the STL/model.config/model.sdf/world files are written |
| `model_name` | `map` | Gazebo model name prefix / filenames |
| `spawn_in_gazebo` | `true` | Wipe + spawn via gazebo_ros services after export |
| `entities_to_keep` | `["ground_plane", "burger"]` | Entities the wipe sweep must not delete (the floor + the robot) |
| `respawn_poll_interval_sec` | `0.25` | How often to re-check `/get_model_list` while confirming a delete/spawn |
| `respawn_poll_timeout_sec` | `5.0` | How long to wait before giving up and proceeding anyway |

## Dependencies

`rclpy`, `nav_msgs`, `gazebo_msgs`, `numpy`, `opencv-python`
