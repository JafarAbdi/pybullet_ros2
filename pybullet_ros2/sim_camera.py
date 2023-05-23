"""Contains APIs to create in sim camera images."""

# Copied from pybullet_envs/minitaur/vision/sim_camera.py

import enum

import numpy as np

DEFAULT_TARGET_DISTANCE = 2
DEFAULT_FOV = 60
DEFAULT_OPENGL_FRUSTUM_FAR = 10.0
DEFAULT_OPENGL_FRUSTUM_NEAR = 0.01


class CameraMode(enum.Enum):
    """Different modes that the camera can operate in."""

    RGB = 1
    DEPTH = 2
    RGBD = 3
    # The point cloud in the world frame, where z-axis is the up direction, x-axis
    # is the front direction, and y axis is the left direction.
    POINTCLOUD_WORLD_FRAME = 4
    # The point cloud in the robot's camera frame, where z-axis is the front
    # direction, x-axis is right direction, and y-axis is the downwards
    # direction.
    POINTCLOUD_ROBOT_FRAME = 5


def from_opengl_depth_to_distance(
    depth: np.ndarray,
    near: float,
    far: float,
) -> np.ndarray:
    """Converts the OpenGL depth to distance."""
    return far * near / (far - (far - near) * depth)


# https://github.com/bulletphysics/bullet3/issues/1924#issuecomment-1091876325
def get_point_cloud(
    depth: np.ndarray,
    width: int,
    height: int,
    view_matrix: list[float],
    proj_matrix: list[float],
) -> np.ndarray:
    """Get the point cloud from the depth image."""
    # based on https://stackoverflow.com/questions/59128880/getting-world-coordinates-from-opengl-depth-buffer
    # "infinite" depths will have a value close to 1

    # create a 4x4 transform matrix that goes from pixel coordinates (and depth values) to world coordinates
    proj_matrix = np.asarray(proj_matrix).reshape([4, 4], order="F")
    view_matrix = np.asarray(view_matrix).reshape([4, 4], order="F")
    tran_pix_world = np.linalg.inv(np.matmul(proj_matrix, view_matrix))

    # create a grid with pixel coordinates and depth values
    y, x = np.mgrid[-1 : 1 : 2 / height, -1 : 1 : 2 / width]
    y *= -1.0
    x, y, z = x.reshape(-1), y.reshape(-1), depth.reshape(-1)
    h = np.ones_like(z)

    pixels = np.stack([x, y, z, h], axis=1)
    # filter out "infinite" depths
    pixels = pixels[z < 0.99]  # noqa: PLR2004
    pixels[:, 2] = 2 * pixels[:, 2] - 1

    # turn pixels to world coordinates
    points = np.matmul(tran_pix_world, pixels.T).T
    points /= points[:, 3:4]
    return points[:, :3]


def create_camera_image(
    pybullet_client,  # noqa: ANN001
    camera_position: list[float],
    camera_orientation: list[float],
    resolution: tuple[int, int],
    projection_mat: list[float],
) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """Creates a simulated camera image.

    Args:
      pybullet_client: A pybullet client.
      camera_position: A list of three floats. The absolute location of the camera
        in the simulated world.
      camera_orientation: A list of four floats. The orientation of the camera in
        world frame, in quaternion.
      resolution: A list of two integers. The horizontal and vertical resolution
        of the camera image in pixels.
      projection_mat: A list of 16 floats. The OpenGL projection matrix, in row
        major.

    Returns:
      A tuple containing the image resolution and the array for the sythesized RGB
      camera image.
    """
    camera_orientation_mat = pybullet_client.getMatrixFromQuaternion(camera_orientation)
    camera_position = [camera_position[0], camera_position[1], camera_position[2]]

    target_point = [0.0, 0.0, DEFAULT_TARGET_DISTANCE]  # w.r.t. camera frame

    target_position = (
        np.dot(
            [
                [
                    camera_orientation_mat[0],
                    camera_orientation_mat[1],
                    camera_orientation_mat[2],
                ],
                [
                    camera_orientation_mat[3],
                    camera_orientation_mat[4],
                    camera_orientation_mat[5],
                ],
                [
                    camera_orientation_mat[6],
                    camera_orientation_mat[7],
                    camera_orientation_mat[8],
                ],
            ],
            target_point,
        )
        + camera_position
    )
    view_mat = pybullet_client.computeViewMatrix(
        camera_position,
        target_position,
        [1, 0, 0],
    )
    _, _, rgba, depth, _ = pybullet_client.getCameraImage(
        resolution[0],
        resolution[1],
        viewMatrix=view_mat,
        projectionMatrix=projection_mat,
        renderer=pybullet_client.ER_BULLET_HARDWARE_OPENGL,
        flags=pybullet_client.ER_NO_SEGMENTATION_MASK,
    )
    return (
        rgba,
        depth,
        get_point_cloud(depth, resolution[0], resolution[1], view_mat, projection_mat),
    )


class MountedCamera:
    """A camera that is fixed to a robot's body part.

    Attributes:
      resolution: A 2-tuple (width, height) that represents the resolution of the
        camera.
      fov_degree: A floating point value that represents the field of view of the
        camera in the vertical direction. The unit is degree.
    """

    def __init__(  # noqa: PLR0913
        self,
        pybullet_client,  # noqa: ANN001
        body_id: int,
        parent_link_id: int,
        relative_translation: tuple[float, float, float],
        relative_rotation: tuple[float, float, float, float],
        resolution: tuple[int, int],
        fov_degree: int = DEFAULT_FOV,
        depth_lower_limit: float = DEFAULT_OPENGL_FRUSTUM_NEAR,
        depth_upper_limit: float = DEFAULT_OPENGL_FRUSTUM_FAR,
    ) -> None:
        """Initializes the MounteeCamera class.

        Args:
          pybullet_client: A BulletClient instance.
          body_id: Integer. The unique body ID returned from loadURDF in pybullet.
          parent_link_id: Integer. The camera is rigidly attached to a body link
            specified by this ID. For example, camers may be mounted to the base or
            other moving parts of the robot.
          relative_translation: A list of three floats. The relative translation
            between the center of the camera and the link.
          relative_rotation: A list of four floats. The quaternion specifying the
            relative rotation of the camera.
          resolution: A list of two integers.
          fov_degree: The vertical field of view of this camera.
          depth_lower_limit: The lower limit of the depth camera.
          depth_upper_limit: The upper limit of the depth camera.
        """
        self._pybullet_client = pybullet_client
        self._body_id = body_id
        self._parent_link_id = parent_link_id
        self._relative_translation = relative_translation
        self._relative_rotation = relative_rotation
        self._resolution = resolution
        self._fov_degree = fov_degree
        self._near = depth_lower_limit
        self._far = depth_upper_limit
        self._prev_depth = None
        width, height = resolution
        self._projection_mat = self._pybullet_client.computeProjectionMatrixFOV(
            fov_degree,
            float(width) / float(height),
            self._near,
            self._far,
        )

    def render_image(self) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
        """Retrieves the camera image.

        Returns:
          A simulated view from the camera's perspective, the depth values, and the 3D point cloud.
        """
        pos = []
        ori = []
        if self._parent_link_id == -1:
            pos, ori = self._pybullet_client.getBasePositionAndOrientation(
                self._body_id,
            )
        else:
            parent_link_state = self._pybullet_client.getLinkState(
                self._body_id,
                self._parent_link_id,
                computeForwardKinematics=True,
            )
            pos = parent_link_state[0]
            ori = parent_link_state[1]

        transform = self._pybullet_client.multiplyTransforms(
            pos,
            ori,
            self._relative_translation,
            self._relative_rotation,
        )

        rgba, depth, point_cloud = create_camera_image(
            pybullet_client=self._pybullet_client,
            camera_position=transform[0],
            camera_orientation=transform[1],
            resolution=self._resolution,
            projection_mat=self._projection_mat,
        )

        # Converts from OpenGL depth map to a distance map (unit: meter).
        distances = from_opengl_depth_to_distance(
            depth,
            self._near,
            self._far,
        )

        return rgba[:, :, 0:3], distances, point_cloud

    @property
    def resolution(self) -> tuple[int, int]:
        """Returns the resolution of the camera."""
        return self._resolution

    @property
    def fov_degree(self) -> int:
        """Returns the field of view of the camera."""
        return self._fov_degree