# Apps

First you need to create a catkin workspace.

```bash
source /opt/ros/kinetic/setup.bash
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws
catkin init
```

Move the code to ~/catkin_ws/src

Build the sample apps:

```bash
cd ~/catkin_ws
catkin build
```

Run the sample app:

```bash
source ~/catkin_ws/devel/setup.bash
export ROS_MASTER_URI=http://<VIPER_IP>:11311
export ROS_IP=<HOST_IP>

rosrun cvm_follow_me cvm_follow_me

```
