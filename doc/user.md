# User Guide

## Demo

This package provides a demo launch file that shows how to use the pybullet_ros2_node to simulate a robot in pybullet. The demo launch file loads the panda robot with a box in the scene and runs the RViz & MoveIt2's MoveGroup to control the robot in the simulation.

```bash
ros2 launch pybullet_ros2 demo.launch.py enable_gui:=true # Or enable_gui:=false if you want to run the simulation without the GUI
```

## Pybullet ROS 2

This package provides a node that runs a pybullet simulator and publishes the joint states and camera images to ROS 2 topics. It also subscribes to the joint commands topic to control the robot in the simulation.

The node have declare two parameters: 1- robot_description 2- enable_gui. The robot_description parameter is the URDF file of the robot that you want to simulate. The enable_gui parameter is a boolean that determines whether to run the simulation with or without the GUI. The default value is `True`.

```python
pybullet_node = Node(
    package="pybullet_ros2",
    executable="pybullet_ros2_node",
    parameters=[
        {"robot_description": "..."},
        {"enable_gui": LaunchConfiguration("enable_gui")},
    ],
)
```

If you use `MoveItConfigsBuilder` to load your robot's URDF file, you can use it directly to load the robot in pybullet.

```python
moveit_config = MoveItConfigsBuilder("MY_ROBOT").to_moveit_configs()

pybullet_node = Node(
    package="pybullet_ros2",
    executable="pybullet_ros2_node",
    parameters=[
        moveit_config.robot_description,
        {"enable_gui": LaunchConfiguration("enable_gui")},
    ],
)
```

### ROS 2 Control Plugin

To run your robot with pybullet_ros2_node, you need to configure your robot's URDF file to use `topic_based_ros2_control/TopicBasedSystem` as the ros2_control plugin, for more information about the plugin, please refer to the [plugin documentation](https://github.com/PickNikRobotics/topic_based_ros2_control/blob/main/doc/user.md).

Example URDF configuration:

```xml
<ros2_control name="PybulletPanda" type="system">
   <hardware>
      <plugin>topic_based_ros2_control/TopicBasedSystem</plugin>
      <param name="joint_commands_topic">/pybullet_joint_commands</param> <!-- Should be the same as the joint_commands_topic_name in the pybullet tag -->
      <param name="joint_states_topic">/pybullet_joint_states</param> <!-- Should be the same as the joint_states_topic_name in the pybullet tag -->
   </hardware>
   ...
</ros2_control>
```

### Pybullet URDF Tag

The `pybullet` tag is used to configure the pybullet simulation. You could have multiple cameras in your simulation, and each camera needs to be configured in the URDF file. The `pybullet` tag is optional, and if it is not present, the default values will be used for the joint states/commands topics with no cameras.

```xml
<pybullet>
   <joint_commands_topic_name>joint_commands</joint_commands_topic_name> <!-- default: pybullet_joint_commands -->
   <joint_states_topic_name>joint_states</joint_states_topic_name> <!-- default: pybullet_joint_states -->
   <!-- The name attribute and the frame_name tag are required for the camera tag -->
   <camera name="wrist_camera">
      <frame_name>panda_hand</frame_name>
      <mode>rgbd</mode> <!-- Optional tag defaults to only rgb -- supported values: rgb, rgbd, depth, and point_cloud -->
      <mode>point_cloud</mode>
      <width>160</width> <!-- Optional tag -- default: 640 -->
      <height>120</height> <!-- Optional tag -- default: 480 -->
      <field_of_view>45</field_of_view> <!-- Optional tag -- default: 60 -->
      <near_plane_distance>0.01</near_plane_distance> <!-- Optional tag -- default: 0.01 -->
      <far_plane_distance>100</far_plane_distance> <!-- Optional tag -- default: 10.0 -->
      <translation_from_link x="0.1" y="0.0" z="0.0" /> <!-- Optional tag -- default: x=0.0 y=0.0 z=0.0 -->
      <rotation_from_link x="0.0" y="0.0" z="0.0" w="1.0" /> <!-- Optional tag -- default: x=0.0 y=0.0 z=0.0 w=1.0-->
   </camera>
</pybullet>
```
