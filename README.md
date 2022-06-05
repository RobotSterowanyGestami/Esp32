
# Esp32

Espressif ESP32 micro-ROS node that read IMU (MPU6050) measurements. It uses Arduino framework and the repository is complete PlatformIO project.

## Table of Contents
1. [micro-ROS installation](#micro-ros-installation)
- [micro-ROS utilities installation](#micro-ros-utilities-installation)
- [Create and run micro-ROS agent](#create-and-run-micro-ros-agent)
2. [ESP32 firmware](#esp32-firmware)
3. [Communication with ESP32](#communication-with-esp32)


## micro-ROS installation
This chapter explains how to install micro-ROS utilities, create and setup firmware for Espressif ESP32 and create micro-ROS agent.

### micro-ROS utilities installation
It is obligatory to have ROS2 installed. Detailed instructions are available [here](https://docs.ros.org/en/galactic/Installation.html). When ROS2 is installed it is possible to install micro-ROS. To do so, follow steps described below:

 1. Source ROS2 installation

```bash
source /opt/ros/$ROS_DISTRO/setup.bash
```

 2. Create a workspace and download the micro-ROS utilities
```bash
mkdir microros_ws
cd microros_ws
git clone -b $ROS_DISTRO https://github.com/micro-ROS/micro_ros_setup.git src/micro_ros_setup
```

3. Update dependencies with rosdep
```bash
sudo apt update && rosdep update
rosdep install --from-paths src --ignore-src -y
```

4. Install pip
```bash
sudo apt-get install python3-pip
```

5. Build and source micro-ROS utilities
```bash
colcon build
source install/local_setup.bash
```

### Create and run micro-ROS agent
Make sure that micro-ROS utilities installation step is complete and both ROS2 and micro-ROS are sourced. Then follow these steps:
1. Create micro-ROS agent
```bash
ros2 run micro_ros_setup create_agent_ws.sh
```
2. Build and source micro-ROS agent
```bash
ros2 run micro_ros_setup build_agent.sh
source install/local_setup.bash
```
3. Run micro-ROS agent
```bash
ros2 run micro_ros_agent micro_ros_agent udp -p 8888
```

## ESP32 firmware

In order to upload firmware to ESP32, clone this repository into your computer. Open it with PlatformIO, then build and upload with PlatformIO's tools.

## Communication with ESP32
The node is sending IMU measurements on /hal_imu topic. Type of message is sensor_msgs/Imu, which contains:
- std_msgs/Header header - not used
- geometry_msgs/Quaternion orientation - filled with zeroes; MPU6050 doesn't support orientation data
- float64[9] orientation_covariance - -1 on beginning, rest filled with zeroes; MPU6050 doesn't support orientation data
- geometry_msgs/Vector3 angular_velocity - gyroscope data
- float64[9] angular_velocity_covariance - filled with zeroes; covariance unknown
- geometry_msgs/Vector3 linear_acceleration - accelerometer data
- float64[9] linear_acceleration_covariance - filled with zeroes; covariance unknown
