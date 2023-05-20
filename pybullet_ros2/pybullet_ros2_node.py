#!/usr/bin/env python3

from threading import Thread
import time
from typing import Final
import pybullet_data

import numpy as np
import pybullet as p
import rclpy
from cv_bridge import CvBridge
from rcl_interfaces.msg import ParameterDescriptor, ParameterType
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import qos_profile_system_default
from sensor_msgs.msg import CameraInfo, Image, JointState

from pybullet_ros2.sim_camera import MountedCamera
from pybullet_ros2.utils import PybulletConfig
import pkgutil


JOINT_STATE_POSITION_INDEX: Final = 0
JOINT_STATE_VELOCITY_INDEX: Final = 1
JOINT_INFO_NAME_INDEX: Final = 1
JOINT_INFO_LINK_NAME_INDEX: Final = 12


class PyBulletNode(Node):
    def __init__(self) -> None:
        super().__init__("pybullet_node")
        self.declare_parameter(
            "robot_description",
            descriptor=ParameterDescriptor(type=ParameterType.PARAMETER_STRING),
        )

        self.pybullet_config = PybulletConfig.from_robot_description(
            self.get_parameter("robot_description").value,
        )

        # use cv_bridge ros to convert cv matrix to ros format
        self.image_bridge = CvBridge()
        # This gives ~124HZ https://github.com/bulletphysics/bullet3/blob/master/examples/pybullet/examples/eglRenderTest.py
        p.connect(p.GUI)
        # p.setAdditionalSearchPath(pybullet_data.getDataPath())

        # egl = pkgutil.get_loader("eglRenderer")
        # if egl:
        #     pluginId = p.loadPlugin(egl.get_filename(), "_eglRendererPlugin")
        # else:
        #     pluginId = p.loadPlugin("eglRendererPlugin")
        # print("pluginId=", pluginId)
        p.configureDebugVisualizer(p.COV_ENABLE_GUI, 0)
        p.configureDebugVisualizer(p.COV_ENABLE_SEGMENTATION_MARK_PREVIEW, 0)
        p.configureDebugVisualizer(p.COV_ENABLE_DEPTH_BUFFER_PREVIEW, 0)
        p.configureDebugVisualizer(p.COV_ENABLE_RGB_BUFFER_PREVIEW, 0)

        p.resetSimulation()
        p.setGravity(0, 0, -9.81)
        self.robot_id = p.loadURDF(
            self.pybullet_config.robot_description,
            useFixedBase=True,
        )
        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        p.loadURDF("plane_transparent.urdf", useMaximalCoordinates=True)
        ### DEMO ###
        shift = [0, -0.02, 0]
        meshScale = [0.1, 0.1, 0.1]
        # the visual shape and collision shape can be re-used by all createMultiBody instances (instancing)
        visualShapeId = p.createVisualShape(
            shapeType=p.GEOM_MESH,
            fileName="duck.obj",
            rgbaColor=[1, 1, 1, 1],
            specularColor=[0.4, 0.4, 0],
            visualFramePosition=shift,
            meshScale=meshScale,
        )
        collisionShapeId = p.createCollisionShape(
            shapeType=p.GEOM_MESH,
            fileName="duck_vhacd.obj",
            collisionFramePosition=shift,
            meshScale=meshScale,
        )

        rangex = 3
        rangey = 3
        for i in range(rangex):
            for j in range(rangey):
                p.createMultiBody(
                    baseMass=1,
                    baseInertialFramePosition=[0, 0, 0],
                    baseCollisionShapeIndex=collisionShapeId,
                    baseVisualShapeIndex=visualShapeId,
                    basePosition=[
                        0.5 + ((-rangex / 2) + i) * meshScale[0] * 2,
                        (-rangey / 2 + j) * meshScale[1] * 2,
                        0,
                    ],
                    useMaximalCoordinates=True,
                )
        ### DEMO ###
        self._joint_names_to_ids = {}
        self._link_names_to_ids = {}
        for joint_index in range(p.getNumJoints(self.robot_id)):
            joint_info = p.getJointInfo(self.robot_id, joint_index)
            self._joint_names_to_ids[
                joint_info[JOINT_INFO_NAME_INDEX].decode("utf-8")
            ] = joint_index
            self._link_names_to_ids[
                joint_info[JOINT_INFO_LINK_NAME_INDEX].decode("utf-8")
            ] = joint_index
        self._cameras = []
        self._camera_image_publishers = []
        self._camera_info_publishers = []
        for camera in self.pybullet_config.cameras:
            if self._link_names_to_ids.get(camera.frame_name) is None:
                msg = f"Camera frame {camera.frame_name} does not exist in URDF"
                raise ValueError(
                    msg,
                )

            self._cameras.append(
                MountedCamera(
                    p,
                    self.robot_id,
                    self._link_names_to_ids[camera.frame_name],
                    camera.translation_from_link,
                    camera.rotation_from_link,
                    (camera.width, camera.height),
                    camera.field_of_view,
                ),
            )
            self._camera_image_publishers.append(
                self.create_publisher(
                    Image,
                    f"/{camera.name}/{camera.image_topic_name}",
                    qos_profile=qos_profile_system_default,
                ),
            )
            self._camera_info_publishers.append(
                self.create_publisher(
                    CameraInfo,
                    f"/{camera.name}/{camera.camera_info_topic_name}",
                    qos_profile=qos_profile_system_default,
                ),
            )

        self.joint_state_publisher = self.create_publisher(
            JointState,
            self.pybullet_config.joint_states_topic_name,
            10,
        )
        self.joint_command_subscriber = self.create_subscription(
            JointState,
            self.pybullet_config.joint_commands_topic_name,
            self.joint_command_callback,
            10,
        )

        self._last_joint_command = None

    def step(self):
        rclpy.spin_once(self, timeout_sec=0.0)
        if self._last_joint_command is not None:
            for joint_name, _joint_index in self._joint_names_to_ids.items():
                joint_indices = []
                joint_positions = []
                for joint_name, joint_position in zip(
                    self._last_joint_command.name,
                    self._last_joint_command.position,
                ):
                    joint_indices.append(self._joint_names_to_ids[joint_name])
                    joint_positions.append(joint_position)
                p.setJointMotorControlArray(
                    self.robot_id,
                    joint_indices,
                    p.POSITION_CONTROL,
                    joint_positions,
                )
        # TODO: Publish clock?
        p.stepSimulation()
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        for joint_name, joint_index in self._joint_names_to_ids.items():
            msg.name.append(joint_name)
            joint_state = p.getJointState(self.robot_id, joint_index)
            msg.position.append(joint_state[JOINT_STATE_POSITION_INDEX])
            msg.velocity.append(joint_state[JOINT_STATE_VELOCITY_INDEX])
        self.joint_state_publisher.publish(msg)

        for camera, mounted_camera, image_publisher, _camera_info_publisher in zip(
            self.pybullet_config.cameras,
            self._cameras,
            self._camera_image_publishers,
            self._camera_info_publishers,
        ):
            image_msg = Image()
            image_msg.header.stamp = self.get_clock().now().to_msg()
            image_msg.header.frame_id = camera.frame_name
            image_msg.height = camera.height
            image_msg.width = camera.width
            image_msg.encoding = "rgb8"
            image_msg.is_bigendian = False
            image_msg.step = camera.width * 3
            start = time.time()
            image_msg.data = self.image_bridge.cv2_to_imgmsg(
                mounted_camera.render_image().astype(np.uint8),
            ).data
            self.get_logger().error(f"Time taken for image: {time.time() - start}")
            image_publisher.publish(image_msg)

            # Copilot generated this :D

    def joint_command_callback(self, msg):
        self._last_joint_command = msg


def main():
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
        # Will this make simulation slower than real time? The default is 240Hz
        # Max hz I can achieve is 33.3HZ
        rate = rate_node.create_rate(240)
        while p.isConnected() and rclpy.ok():
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
