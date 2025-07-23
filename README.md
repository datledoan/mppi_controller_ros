# Model Predictive Path Integral Controller

Port of the [nav2_mppi_controller](https://github.com/ros-navigation/navigation2/tree/main/nav2_mppi_controller) to ROS1. Applicable for both [move_base](http://wiki.ros.org/move_base) and [move_base_flex](http://wiki.ros.org/move_base_flex). Tested with ROS-Noetic.

# Installation
* Install move_base_flex
    ```sh
    sudo apt install ros-noetic-mbf-costmap-nav
    ```
* Clone code
    ```sh
    cd your_ws/src
    git clone https://github.com/datledoan/mppi_controller_ros.git
    catkin build
    ```
* See its [Configuration Guide Page](https://docs.nav2.org/configuration/packages/configuring-mppic.html)

# Demo

![](media/mppi_demo.gif)

# Reference
- [nav2_mppi_controller](https://github.com/ros-navigation/navigation2/tree/main/nav2_mppi_controller)
