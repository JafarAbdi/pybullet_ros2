"""A ROS 2 node that runs a pybullet simulation."""

from dataclasses import dataclass
from threading import Thread
from typing import Final

import numpy as np
import pybullet
import pybullet_data
import rclpy
from cv_bridge import CvBridge
from rcl_interfaces.msg import ParameterDescriptor, ParameterType
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.publisher import Publisher
from rclpy.qos import qos_profile_system_default
from sensor_msgs.msg import Image, JointState, PointCloud2, PointField

from pybullet_ros2.sim_camera import (
    CameraMode,
    MountedCamera,
)
from pybullet_ros2.utils import CameraConfig, PybulletConfig

JOINT_STATE_POSITION_INDEX: Final = 0
JOINT_STATE_VELOCITY_INDEX: Final = 1
JOINT_INFO_NAME_INDEX: Final = 1
JOINT_INFO_LINK_NAME_INDEX: Final = 12


@dataclass(slots=True)
class Camera:
    """A class that represents a pybullet camera."""

    config: CameraConfig
    mounted_camera: MountedCamera
    publishers: dict[CameraMode, Publisher]


class PyBulletNode(Node):
    """A ROS 2 node that will publish joint states and images from a pybullet simulation."""

    def __init__(self) -> None:
        """Initialize the node and the pybullet simulation."""
        super().__init__("pybullet_node")
        self.declare_parameter(
            "robot_description",
            descriptor=ParameterDescriptor(type=ParameterType.PARAMETER_STRING),
        )
        self.declare_parameter(
            "enable_gui",
            descriptor=ParameterDescriptor(type=ParameterType.PARAMETER_BOOL),
        )

        pybullet_config = PybulletConfig.from_robot_description(
            self.get_parameter("robot_description").value,
        )

        self.image_bridge = CvBridge()
        if self.get_parameter("enable_gui").value:
            pybullet.connect(pybullet.GUI)
            pybullet.setAdditionalSearchPath(pybullet_data.getDataPath())
        else:
            pybullet.connect(pybullet.DIRECT)
            # This gives ~124HZ https://github.com/bulletphysics/bullet3/blob/master/examples/pybullet/examples/eglRenderTest.py
            import pkgutil

            egl = pkgutil.find_loader("eglRenderer")
            egl = pkgutil.get_loader("eglRenderer")

            pybullet.setAdditionalSearchPath(pybullet_data.getDataPath())
            pybullet.loadPlugin(egl.get_filename(), "_eglRendererPlugin")
            pybullet.configureDebugVisualizer(pybullet.COV_ENABLE_RENDERING, 0)

        pybullet.configureDebugVisualizer(pybullet.COV_ENABLE_GUI, 0)
        pybullet.configureDebugVisualizer(
            pybullet.COV_ENABLE_SEGMENTATION_MARK_PREVIEW,
            0,
        )
        pybullet.configureDebugVisualizer(pybullet.COV_ENABLE_DEPTH_BUFFER_PREVIEW, 0)
        pybullet.configureDebugVisualizer(pybullet.COV_ENABLE_RGB_BUFFER_PREVIEW, 0)

        pybullet.resetSimulation()
        pybullet.setGravity(0, 0, -9.81)
        self.robot_id = pybullet.loadURDF(
            pybullet_config.robot_description,
            useFixedBase=True,
        )
        pybullet.loadURDF("plane.urdf")

        self._joint_names_to_ids = {}
        self._link_names_to_ids = {}
        for joint_index in range(pybullet.getNumJoints(self.robot_id)):
            joint_info = pybullet.getJointInfo(self.robot_id, joint_index)
            self._joint_names_to_ids[
                joint_info[JOINT_INFO_NAME_INDEX].decode("utf-8")
            ] = joint_index
            self._link_names_to_ids[
                joint_info[JOINT_INFO_LINK_NAME_INDEX].decode("utf-8")
            ] = joint_index

        self._last_joint_command = None
        self.joint_state_publisher = self.create_publisher(
            JointState,
            pybullet_config.joint_states_topic_name,
            10,
        )
        self.joint_command_subscriber = self.create_subscription(
            JointState,
            pybullet_config.joint_commands_topic_name,
            self.joint_command_callback,
            10,
        )
        self._cameras = []
        for camera_config in pybullet_config.cameras:
            if self._link_names_to_ids.get(camera_config.frame_name) is None:
                msg = f"Camera frame {camera_config.frame_name} does not exist in URDF"
                raise ValueError(
                    msg,
                )
            publishers = {}
            for camera_mode in camera_config.modes:
                match camera_mode:
                    case CameraMode.RGB:
                        publishers[CameraMode.RGB] = self.create_publisher(
                            Image,
                            f"/{camera_config.name}/color/image_raw",
                            qos_profile=qos_profile_system_default,
                        )
                    case CameraMode.RGBD:
                        publishers[CameraMode.RGB] = self.create_publisher(
                            Image,
                            f"/{camera_config.name}/color/image_raw",
                            qos_profile=qos_profile_system_default,
                        )
                        publishers[CameraMode.DEPTH] = self.create_publisher(
                            Image,
                            f"/{camera_config.name}/depth/image_rect_raw",
                            qos_profile=qos_profile_system_default,
                        )
                    case CameraMode.DEPTH:
                        publishers[CameraMode.DEPTH] = self.create_publisher(
                            Image,
                            f"/{camera_config.name}/depth/image_rect_raw",
                            qos_profile=qos_profile_system_default,
                        )
                    case CameraMode.POINTCLOUD_ROBOT_FRAME:
                        publishers[
                            CameraMode.POINTCLOUD_ROBOT_FRAME
                        ] = self.create_publisher(
                            PointCloud2,
                            f"/{camera_config.name}/depth/color/points",
                            qos_profile=qos_profile_system_default,
                        )

            self._cameras.append(
                Camera(
                    config=camera_config,
                    mounted_camera=MountedCamera(
                        pybullet,
                        self.robot_id,
                        self._link_names_to_ids[camera_config.frame_name],
                        camera_config.translation_from_link,
                        camera_config.rotation_from_link,
                        (camera_config.width, camera_config.height),
                        camera_config.field_of_view,
                    ),
                    publishers=publishers,
                ),
            )

    def step(self) -> None:
        """Set the last commanded joint state and step the simulation forward one step then publish the state + images."""
        rclpy.spin_once(self, timeout_sec=0.0)
        if self._last_joint_command is not None:
            joint_indices = []
            joint_positions = []
            for joint_name, joint_position in zip(
                self._last_joint_command.name,
                self._last_joint_command.position,
                strict=True,
            ):
                joint_indices.append(self._joint_names_to_ids[joint_name])
                joint_positions.append(joint_position)
            pybullet.setJointMotorControlArray(
                self.robot_id,
                joint_indices,
                pybullet.POSITION_CONTROL,
                joint_positions,
            )
        pybullet.stepSimulation()
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        for joint_name, joint_index in self._joint_names_to_ids.items():
            msg.name.append(joint_name)
            joint_state = pybullet.getJointState(self.robot_id, joint_index)
            msg.position.append(joint_state[JOINT_STATE_POSITION_INDEX])
            msg.velocity.append(joint_state[JOINT_STATE_VELOCITY_INDEX])
        self.joint_state_publisher.publish(msg)

        for camera in self._cameras:
            rgb_image, depth, point_cloud = camera.mounted_camera.render_image()
            for camera_mode, publisher in camera.publishers.items():
                match camera_mode:
                    case CameraMode.RGB:
                        image_msg = self.image_bridge.cv2_to_imgmsg(
                            rgb_image.astype(np.uint8),
                            "rgb8",
                        )
                        image_msg.header.stamp = self.get_clock().now().to_msg()
                        image_msg.header.frame_id = camera.config.frame_name
                        publisher.publish(image_msg)
                    case CameraMode.DEPTH:
                        image_msg = self.image_bridge.cv2_to_imgmsg(depth)
                        image_msg.header.stamp = self.get_clock().now().to_msg()
                        image_msg.header.frame_id = camera.config.frame_name
                        publisher.publish(image_msg)
                    case CameraMode.POINTCLOUD_ROBOT_FRAME:
                        point_cloud_msg = PointCloud2()
                        point_cloud_msg.header.stamp = self.get_clock().now().to_msg()
                        point_cloud_msg.header.frame_id = "panda_link0"
                        point_cloud_msg.height = 1
                        point_cloud_msg.width = (
                            camera.config.width * camera.config.height
                        )
                        point_cloud_msg.is_dense = True
                        point_cloud_msg.is_bigendian = False
                        point_cloud_msg.point_step = 12
                        point_cloud_msg.row_step = (
                            camera.config.width * point_cloud_msg.point_step
                        )
                        point_cloud_msg.fields = [
                            PointField(
                                name="x",
                                offset=0,
                                datatype=PointField.FLOAT32,
                                count=1,
                            ),
                            PointField(
                                name="y",
                                offset=4,
                                datatype=PointField.FLOAT32,
                                count=1,
                            ),
                            PointField(
                                name="z",
                                offset=8,
                                datatype=PointField.FLOAT32,
                                count=1,
                            ),
                        ]
                        point_cloud_msg.data = point_cloud.astype(np.float32).tobytes()
                        publisher.publish(point_cloud_msg)

    def joint_command_callback(self, msg: JointState) -> None:
        """Callback for the joint command subscriber."""
        self._last_joint_command = msg


def main() -> None:
    """Main script."""
    rclpy.init()
    pybullet_node = PyBulletNode()
    # TODO: Have a separate callback group for the Rate once https://github.com/ros2/rclpy/issues/850 is fixed
    rate_node = Node("rate_node")
    # TODO: Using rclpy.spin causing some issues with wait sets https://github.com/ros2/rclpy/issues/1008
    executor = SingleThreadedExecutor()
    executor.add_node(rate_node)
    rate_thread = Thread(target=executor.spin, daemon=True)
    rate_thread.start()

    try:
        rate = rate_node.create_rate(240)
        while pybullet.isConnected() and rclpy.ok():
            pybullet_node.step()
            rate.sleep()
    except KeyboardInterrupt:
        pass
    finally:
        pybullet_node.destroy_node()
        rclpy.shutdown()
        executor.remove_node(rate_node)
        rate_thread.join()


if __name__ == "__main__":
    main()
