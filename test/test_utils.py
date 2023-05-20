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
                <frame_name>camera2</frame_name>
                <translation_from_link x="0.1" y="2.0" z="0.5"/>
                <rotation_from_link x="0.0" y="0.0" z="0.707" w="0.707"/>
            </camera>
        </pybullet>
    </robot>
    """
    config = PybulletConfig.from_robot_description(robot)
    assert len(config.cameras) == 2
    assert config.cameras[0].name == "camera2"
    assert config.cameras[0].frame_name == "camera1"
    assert config.cameras[0].camera_info_topic_name == "camera_info"
    assert config.cameras[0].image_topic_name == "image_raw"
    assert config.cameras[1].name == "camera1"
    assert config.cameras[1].frame_name == "camera2"
    assert config.cameras[1].camera_info_topic_name == "camera_info"
    assert config.cameras[1].image_topic_name == "image_raw"
    assert config.joint_states_topic_name == "pybullet_joint_states"
    assert config.joint_commands_topic_name == "pybullet_joint_commands"
    assert config.cameras[0].translation_from_link == (0.0, 0.0, 0.0)
    assert config.cameras[0].rotation_from_link == (0.0, 0.0, 0.0, 1.0)
    assert config.cameras[1].translation_from_link == (0.1, 2.0, 0.5)
    assert config.cameras[1].rotation_from_link == (0.0, 0.0, 0.707, 0.707)
