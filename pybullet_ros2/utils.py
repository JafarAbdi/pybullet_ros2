import xml.etree.ElementTree as ET
from dataclasses import dataclass, field
from pathlib import Path
from tempfile import NamedTemporaryFile

from ament_index_python.packages import get_package_share_path
from urdf_parser_py.urdf import URDF
import pybullet_ros2.sim_camera as sim_camera


def normalized_robot_description(file: Path | str):
    """Normalize the robot description file to use absolute paths for the collision and visual tags."""
    if isinstance(file, str):
        robot = URDF.from_xml_string(file)
    elif isinstance(file, Path):
        robot = URDF.from_xml_file(file)

    for _link_index, link in enumerate(robot.links):
        for collision in link.collisions:
            if collision.geometry.filename.startswith("package://"):
                package_name, relative_path = collision.geometry.filename.split(
                    "package://",
                )[1].split("/", 1)
                collision.geometry.filename = (
                    f"file:/{get_package_share_path(package_name)}/{relative_path}"
                )
        for visual in link.visuals:
            if visual.geometry.filename.startswith("package://"):
                package_name, relative_path = visual.geometry.filename.split(
                    "package://",
                )[1].split("/", 1)
                visual.geometry.filename = (
                    f"file:/{get_package_share_path(package_name)}/{relative_path}"
                )

    with NamedTemporaryFile(
        mode="w",
        prefix="pybullet_ros2_",
        delete=False,
    ) as parsed_file:
        parsed_file_path = parsed_file.name
        parsed_file.write(robot.to_xml_string())
        return parsed_file_path


@dataclass(slots=True)
class Camera:
    # Required
    name: str
    # Required
    frame_name: str
    image_topic_name: str
    camera_info_topic_name: str
    width: int
    height: int
    field_of_view: float
    near_plane_distance: float
    far_plane_distance: float
    translation_from_link: list[float]
    rotation_from_link: list[float]


def get_tag(tree, tag_name, default=None):
    if (tag_value := tree.find(tag_name)) is None:
        if default is None:
            raise RuntimeError(f"Missing a required tag value for {tag_name}")
        return default
    return tag_value.text


def get_attribute(tree, attribute_name, default=None):
    if (attribute_value := tree.get(attribute_name)) is None:
        if default is None:
            raise RuntimeError(
                f"Missing a required attribute value for {attribute_name}"
            )
        return default
    return attribute_value


def parse_translation_from_link(tree):
    if (translation_from_link := tree.find("translation_from_link")) is None:
        return (0.0, 0.0, 0.0)
    return (
        float(translation_from_link.get("x")),
        float(translation_from_link.get("y")),
        float(translation_from_link.get("z")),
    )


def parse_rotation_from_link(tree):
    if (rotation_from_link := tree.find("rotation_from_link")) is None:
        return (0.0, 0.0, 0.0, 1.0)
    return (
        float(rotation_from_link.get("x")),
        float(rotation_from_link.get("y")),
        float(rotation_from_link.get("z")),
        float(rotation_from_link.get("w")),
    )


@dataclass(slots=True)
class PybulletConfig:
    cameras: list[Camera] = field(default_factory=list)
    joint_commands_topic_name: str = "pybullet_joint_commands"
    joint_states_topic_name: str = "pybullet_joint_states"
    robot_description: Path | None = None

    @classmethod
    def from_robot_description(cls, robot_description: str | Path):
        if isinstance(robot_description, str):
            tree = ET.fromstring(robot_description)
        elif isinstance(robot_description, Path):
            tree = ET.parse(robot_description)
        if (pybullet_tag := tree.find("pybullet")) is None:
            return cls()
        cameras = []
        for camera_tag in pybullet_tag.findall("camera"):
            cameras.append(
                Camera(
                    name=get_attribute(camera_tag, "name"),
                    frame_name=get_tag(camera_tag, "frame_name"),
                    image_topic_name=get_tag(
                        camera_tag, "image_topic_name", "image_raw"
                    ),
                    camera_info_topic_name=get_tag(
                        camera_tag, "camera_info_topic_name", "camera_info"
                    ),
                    width=int(get_tag(camera_tag, "width", 640)),
                    height=int(get_tag(camera_tag, "height", 480)),
                    field_of_view=float(
                        get_tag(camera_tag, "field_of_view", sim_camera._DEFAULT_FOV)
                    ),
                    near_plane_distance=float(
                        get_tag(
                            camera_tag,
                            "near_plane_distance",
                            sim_camera._DEFAULT_OPENGL_FRUSTUM_NEAR,
                        )
                    ),
                    far_plane_distance=float(
                        get_tag(
                            camera_tag,
                            "far_plane_distance",
                            sim_camera._DEFAULT_OPENGL_FRUSTUM_FAR,
                        ),
                    ),
                    translation_from_link=parse_translation_from_link(camera_tag),
                    rotation_from_link=parse_rotation_from_link(camera_tag),
                )
            )

        joint_states_topic_name = "pybullet_joint_states"
        if (
            joint_states_topic_name_tag := pybullet_tag.find("joint_states_topic_name")
        ) is not None:
            joint_states_topic_name = joint_states_topic_name_tag.text
        joint_commands_topic_name = "pybullet_joint_commands"
        if (
            joint_commands_topic_name_tag := pybullet_tag.find(
                "joint_commands_topic_name",
            )
        ) is not None:
            joint_commands_topic_name = joint_commands_topic_name_tag.text

        return cls(
            cameras=cameras,
            joint_states_topic_name=joint_states_topic_name,
            joint_commands_topic_name=joint_commands_topic_name,
            robot_description=normalized_robot_description(robot_description),
        )
