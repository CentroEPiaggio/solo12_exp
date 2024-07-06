# SOLO 12 Experiments

## Installation

Either use Docker or copy the `src` directory in your ROS 2 workspace. The repo has been tested in ROS 2 humble.

To use the Docker installation, follow the Preliminaries instructions [here](https://github.com/ddebenedittis/docker_ros_nvidia?tab=readme-ov-file#preliminaries). Afterward, build (once) and run the Docker image with the following commands:
- Build:
    ```shell
    ./build.bash [-r]
    ```
- Run:
    ```shell
    ./run.bash
    ```

## Usage

### SOLO12 Interface

Verify that in `src/solo12_interface/solo12_interface/src/wb_interface.cpp`, the defined `CONNECTION_MODE` is the same as the output of the `ip a` (the ethernet one) command executed in the shell. Otherwise, change it and rebuild the package.

To launch the SOLO12 interface:
1. Connect the robot to the computer (either via Ethernet or Wireless).
2. Enter superuser mode:
    ```shell
    sudo su
    ```
3. Afterwards, source the ROS 2 workspace and launch the interface:
    ```shell
    source install/setup.bash
    ros2 launch solo12_sim real_leg_interface.launch.py
    ```

### Feed-Forward Commands Publisher

This package publishes the robot's commands as a feed-forward control signal.
To use it, record your bag files and put them in `src/ff_commands_publisher/bags/`.
More detailed instructions are in the package's README.

The bags should be recorded with
```shell
ros2 bag record --use_sim_time /joint_states
```

- Inspect the bag file (plots the joint positions, velocities, and efforts):
    ```shell
    ros2 run ff_commands_publisher bag_inspector
    ```
- Publish and visualize in RViz SOLO12 motion:
    ```shell
    ros2 launch ff_commands_publisher visualize_solo.launch.py ["bag_filename:='<filename>'"] [rate:=<num>] [use_sim_time:=<true|false>]
    ```
    The "<param_name>:='<param_value>'" is only required when the given filename is a number (e.g. 020) since we want it to be treated as a string.
- Publish the bag JointState messages in the topic `/joint_states`:
    ```shell
    ros2 launch ff_commands_publisher ff_commands_publisher_node.launch.py ["bag_filename:='<filename>'"]
    ```

### Feed-Forward Trajectory Simulation

Simulate the feed-forward trajectory in Gazebo with
```shell
ros2 launch robot_gazebo solo12.launch.py ["bag_filename:='<filename>'"]
```
