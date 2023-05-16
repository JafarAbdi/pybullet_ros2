# Installation

## Build from Source

These instructions assume you are running on Ubuntu 22.04:

1. [Install ROS2 humble](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html). You can stop following along with the tutorial after you complete the section titled: [Environment setup](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html#environment-setup). Make sure you setup your environment with:

```bash
source /opt/ros/humble/setup.bash
```

   > *NOTE:* You may want to add that line to your `~/.bashrc`

1. [Install ROS2 Build Tools](https://docs.ros.org/en/humble/Installation/Ubuntu-Development-Setup.html#install-development-tools-and-ros-tools)

   > *NOTE:* If installing on a fresh OS, run `sudo rosdep init` and `rosdep update` before the install script

1. Create a colcon workspace (*Note:* Feel free to change `~/ws_ros2` to whatever absolute path you want):

```bash
export COLCON_WS=~/ws_ros2/
mkdir -p $COLCON_WS/src
```

1. Get the repo:

```bash
cd $COLCON_WS/src
git clone git@github.com:JafarAbdi/pybullet_ros2.git
```

1. Download the required repositories and install any dependencies:

```bash
cd $COLCON_WS/src
vcs import < pybullet_ros2/pybullet_ros2.repos
rosdep install --ignore-src --from-paths . -y

# Pick a ROS_DOMAIN_ID that doesn't clash with others
echo 'export ROS_DOMAIN_ID='<YOUR-NUMBER> >> ~/.bashrc
```

1. Configure and build the workspace:

```bash
cd $COLCON_WS
colcon build --symlink-install --event-handlers log-
```

1. Source the workspace.

```bash
source $COLCON_WS/install/setup.bash
```

> *Note*: Whenever you open a new terminal be sure to run `source /opt/ros/humble/setup.bash`, `export COLCON_WS=~/ws_ros2`, and `source $COLCON_WS/install/setup.bash`. You may want to add those commands to your `~/.bashrc`
