## Container related

### root user

```bash
docker build -t atinfinity/cudagl:11.8.0-cudnn8-devel-ubuntu22.04 -f Dockerfile.root .
```

## Launch docker container

```bash
sudo ./launch_container.sh
```
### launch container terminal
```bash
sudo ./launch_terminal.sh
```


## Start project
```bash
cd ./workspace
ulimit -n 1024
gz sim building_robot.sdf
ros2 run ros_gz_bridge parameter_bridge /lidar@sensor_msgs/msg/LaserScan@gz.msgs.LaserScan
ros2 run ros_gz_bridge parameter_bridge /cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist
ros2 run ros_gz_bridge parameter_bridge /imu@sensor_msgs/msg/Imu@gz.msgs.IMU
source ./ros2_ws/install/setup.bash
colcon build --symlink-install
ros2 run ros2_course talker
```

## Usefull
- [ROS GZ bridge communication](https://github.com/gazebosim/ros_gz/tree/ros2/ros_gz_bridge)
- [Harmonic tutorials](https://gazebosim.org/docs/harmonic/moving_robot)

2.6-3.2
