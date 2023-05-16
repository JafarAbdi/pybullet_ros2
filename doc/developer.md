# Developers Guide

To make sure you have the latest repos:

      cd $COLCON_WS/src/pybullet_ros2
      git checkout main
      git pull origin main
      cd $COLCON_WS/src
      vcs import < pybullet_ros2/pybullet_ros2.repos
      rosdep install --from-paths . --ignore-src -y

## Setup pre-commit

pre-commit is a tool to automatically run formatting checks on each commit, which saves you from manually running clang-format (or, crucially, from forgetting to run them!).

Install pre-commit like this:

```
pip3 install pre-commit
```

Run this in the top directory of the repo to set up the git hooks:

```
pre-commit install
```

## Testing and Linting

To test the pacakges in pybullet_ros2, use the following command with [colcon](https://colcon.readthedocs.io/en/released/).

    export TEST_PACKAGES="PROJECT_PACKAGE_NAMES"
    colcon build --packages-up-to ${TEST_PACKAGES}
    colcon test --packages-select ${TEST_PACKAGES}
    colcon test-result
