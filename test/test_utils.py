from pybullet_ros2.sim_camera import CameraMode
from pybullet_ros2.utils import PybulletConfig


def test_empty_pybullet_config():
    robot = """
    <robot name="panda">
        <pybullet>
        </pybullet>
    </robot>
    """

    config = PybulletConfig.from_robot_description(robot)
    assert len(config.cameras) == 0


def test_no_pybullet_config():
    robot = """
    <robot name="panda">
    </robot>
    """

    config = PybulletConfig.from_robot_description(robot)
    assert len(config.cameras) == 0


def test_joint_state_command_topic_name():
    robot = """
    <robot name="panda">
        <pybullet>
            <joint_commands_topic_name>joint_commands</joint_commands_topic_name>
            <joint_states_topic_name>joint_states</joint_states_topic_name>
        </pybullet>
    </robot>
    """
    config = PybulletConfig.from_robot_description(robot)
    assert len(config.cameras) == 0
    assert config.joint_commands_topic_name == "joint_commands"
    assert config.joint_states_topic_name == "joint_states"


def test_pybullet_config_from_robot_description():
    robot = """
    <robot name="panda">
        <pybullet>
            <camera name="camera2">
                <frame_name>camera1</frame_name>
            </camera>
            <camera name="camera1">
                <mode>rgbd</mode>
                <mode>point_cloud</mode>
                <frame_name>camera2</frame_name>
                <translation_from_link x="0.1" y="2.0" z="0.5"/>
                <rotation_from_link x="0.0" y="0.0" z="0.707" w="0.707"/>
            </camera>
            <camera name="camera3">
                <mode>depth</mode>
                <frame_name>camera_frame</frame_name>
            </camera>
            <camera name="camera4">
                <mode>point_cloud</mode>
                <frame_name>camera_frame</frame_name>
            </camera>
        </pybullet>
    </robot>
    """
    config = PybulletConfig.from_robot_description(robot)
    assert len(config.cameras) == 4
    assert config.cameras[0].name == "camera2"
    assert config.cameras[0].frame_name == "camera1"
    assert config.cameras[0].modes == [CameraMode.RGB]
    assert config.cameras[1].name == "camera1"
    assert config.cameras[1].frame_name == "camera2"
    assert config.joint_states_topic_name == "pybullet_joint_states"
    assert config.joint_commands_topic_name == "pybullet_joint_commands"
    assert config.cameras[0].translation_from_link == (0.0, 0.0, 0.0)
    assert config.cameras[0].rotation_from_link == (0.0, 0.0, 0.0, 1.0)
    assert config.cameras[1].translation_from_link == (0.1, 2.0, 0.5)
    assert config.cameras[1].rotation_from_link == (0.0, 0.0, 0.707, 0.707)
    assert config.cameras[1].modes == [
        CameraMode.RGBD,
        CameraMode.POINTCLOUD_ROBOT_FRAME,
    ]
    assert config.cameras[2].modes == [CameraMode.DEPTH]
    assert config.cameras[3].modes == [CameraMode.POINTCLOUD_ROBOT_FRAME]
