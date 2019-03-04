# Samples

First you need to create a catkin workspace.

```bash
source /opt/ros/kinetic/setup.bash
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws
catkin init
```

Move the samples to ~/catkin_ws/src

Build the samples:

```bash
cd ~/catkin_ws
catkin build
```

Run a sample:

```bash
source ~/catkin_ws/devel/setup.bash
export ROS_MASTER_URI=http://<VIPER_IP>:11311
rosrun image_streaming image_streaming

```
or

```bash
rosrun disparity_streaming disparity_streaming
rosrun pointcloud_streaming pointcloud_streaming
rosrun laserscan_streaming laserscan_streaming
```
