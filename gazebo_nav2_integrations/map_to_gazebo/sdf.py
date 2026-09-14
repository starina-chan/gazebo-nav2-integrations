"""Pure string builders for the exported model.config / model.sdf / world files.

No file I/O here - callers write the returned strings wherever they need to.
"""


def model_config_xml(model_name):
    return f"""<?xml version="1.0" ?>
<model>
  <name>{model_name}</name>
  <version>1.0</version>
  <sdf version="1.6">model.sdf</sdf>
  <description></description>
</model>
"""


def model_sdf_xml(entity_name, mesh_uri):
    return f"""<?xml version="1.0" ?>
<sdf version="1.6">
  <model name="{entity_name}">
    <static>1</static>
    <link name="link">
      <collision name="collision">
        <pose>0 0 0 0 0 0</pose>
        <geometry>
          <mesh>
            <uri>{mesh_uri}</uri>
          </mesh>
        </geometry>
      </collision>
      <visual name="visual">
        <pose>0 0 0 0 0 0</pose>
        <geometry>
          <mesh>
            <uri>{mesh_uri}</uri>
          </mesh>
        </geometry>
      </visual>
    </link>
  </model>
</sdf>
"""


def world_xml(export_dir):
    return f"""<?xml version="1.0" ?>
<sdf version="1.6">
  <world name="default">
    <include>
      <uri>model://ground_plane</uri>
    </include>
    <include>
      <uri>model://sun</uri>
    </include>
    <include>
      <uri>file://{export_dir}</uri>
    </include>
  </world>
</sdf>
"""
