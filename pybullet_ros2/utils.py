"""Utility functions for the pybullet_ros2 package."""

import xml.etree.ElementTree as ET  # noqa: N817
from dataclasses import dataclass, field
from pathlib import Path
from tempfile import NamedTemporaryFile

from ament_index_python.packages import get_package_share_path
from urdf_parser_py.urdf import URDF, Mesh

from pybullet_ros2 import sim_camera


def normalized_robot_description(file: Path | str) -> str:
    """Normalize the robot description file to use absolute paths for the collision and visual tags."""
    if isinstance(file, str):
        robot = URDF.from_xml_string(file)
    elif isinstance(file, Path):
        robot = URDF.from_xml_file(file)

    for link in robot.links:
        for collision in link.collisions:
            if isinstance(
                collision.geometry,
                Mesh,
            ) and collision.geometry.filename.startswith("package://"):
                package_name, relative_path = collision.geometry.filename.split(
                    "package://",
                )[1].split("/", 1)
                collision.geometry.filename = (
                    f"file:/{get_package_share_path(package_name)}/{relative_path}"
                )
        for visual in link.visuals:
            if isinstance(
                collision.geometry,
                Mesh,
            ) and visual.geometry.filename.startswith("package://"):
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
class CameraConfig:
    """Camera configuration."""

    name: str
    frame_name: str
    width: int
    height: int
    field_of_view: float
    near_plane_distance: float
    far_plane_distance: float
    translation_from_link: list[float]
    rotation_from_link: list[float]
    modes: list[sim_camera.CameraMode]


def get_tag(tree: ET.Element, tag_name: str, default: str | None = None) -> str:
    """Get a tag from the tree."""
    if (tag_value := tree.find(tag_name)) is None:
        if default is None:
            msg = f"Missing a required tag value for {tag_name}"
            raise RuntimeError(msg)
        return default
    return tag_value.text


def get_tags(
    tree: ET.Element,
    tag_name: str,
    default: list[str] | None = None,
) -> list[str]:
    """Get all tags with the given name from the tree."""
    if len(tag_value := tree.findall(tag_name)) == 0:
        if default is None:
            msg = f"Missing a required tag value for {tag_name}"
            raise RuntimeError(msg)
        return default
    return [tag.text for tag in tag_value]


def get_attribute(
    tree: ET.Element,
    attribute_name: str,
    default: str | None = None,
) -> str:
    """Get an attribute from the tree."""
    if (attribute_value := tree.get(attribute_name)) is None:
        if default is None:
            msg = f"Missing a required attribute value for {attribute_name}"
            raise RuntimeError(
                msg,
            )
        return default
    return attribute_value


def parse_translation_from_link(tree: ET.Element) -> tuple[float, float, float]:
    """Parse the translation_from_link tag."""
    if (translation_from_link := tree.find("translation_from_link")) is None:
        return (0.0, 0.0, 0.0)
    return (
        float(translation_from_link.get("x")),
        float(translation_from_link.get("y")),
        float(translation_from_link.get("z")),
    )


def parse_rotation_from_link(tree: ET.Element) -> tuple[float, float, float, float]:
    """Parse the rotation_from_link tag."""
    if (rotation_from_link := tree.find("rotation_from_link")) is None:
        return (0.0, 0.0, 0.0, 1.0)
    return (
        float(rotation_from_link.get("x")),
        float(rotation_from_link.get("y")),
        float(rotation_from_link.get("z")),
        float(rotation_from_link.get("w")),
    )


def camera_mode_from_string(mode: str) -> sim_camera.CameraMode:
    """Convert a string to a CameraMode enum."""
    match mode:
        case "rgb":
            return sim_camera.CameraMode.RGB
        case "depth":
            return sim_camera.CameraMode.DEPTH
        case "rgbd":
            return sim_camera.CameraMode.RGBD
        case "point_cloud":
            return sim_camera.CameraMode.POINTCLOUD_ROBOT_FRAME
    msg = f"Invalid camera mode: {mode}"
    raise RuntimeError(msg)


def get_camera_modes(tree: ET.Element) -> list[sim_camera.CameraMode]:
    """Get the camera modes from the tree."""
    return list(map(camera_mode_from_string, get_tags(tree, "mode", ["rgb"])))


@dataclass(slots=True)
class PybulletConfig:
    """Pybullet configuration."""

    cameras: list[CameraConfig] = field(default_factory=list)
    joint_commands_topic_name: str = "pybullet_joint_commands"
    joint_states_topic_name: str = "pybullet_joint_states"
    robot_description: Path | None = None

    @classmethod
    def from_robot_description(cls, robot_description: str | Path) -> "PybulletConfig":
        """Parse the pybullet tag from the robot description."""
        if isinstance(robot_description, str):
            tree = ET.fromstring(robot_description)  # noqa: S314
        elif isinstance(robot_description, Path):
            tree = ET.parse(robot_description)  # noqa: S314
        if (pybullet_tag := tree.find("pybullet")) is None:
            return cls()
        cameras = [
            CameraConfig(
                name=get_attribute(camera_tag, "name"),
                frame_name=get_tag(camera_tag, "frame_name"),
                width=int(get_tag(camera_tag, "width", 640)),
                height=int(get_tag(camera_tag, "height", 480)),
                field_of_view=float(
                    get_tag(camera_tag, "field_of_view", sim_camera.DEFAULT_FOV),
                ),
                near_plane_distance=float(
                    get_tag(
                        camera_tag,
                        "near_plane_distance",
                        sim_camera.DEFAULT_OPENGL_FRUSTUM_NEAR,
                    ),
                ),
                far_plane_distance=float(
                    get_tag(
                        camera_tag,
                        "far_plane_distance",
                        sim_camera.DEFAULT_OPENGL_FRUSTUM_FAR,
                    ),
                ),
                translation_from_link=parse_translation_from_link(camera_tag),
                rotation_from_link=parse_rotation_from_link(camera_tag),
                modes=get_camera_modes(camera_tag),
            )
            for camera_tag in pybullet_tag.findall("camera")
        ]
        return cls(
            cameras=cameras,
            joint_states_topic_name=get_tag(
                pybullet_tag,
                "joint_states_topic_name",
                "pybullet_joint_states",
            ),
            joint_commands_topic_name=get_tag(
                pybullet_tag,
                "joint_commands_topic_name",
                "pybullet_joint_commands",
            ),
            robot_description=normalized_robot_description(robot_description),
        )
