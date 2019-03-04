Steps before running and/or building FollowAruco sample app:
1. Source CVM workspace
  $ source /opt/rubedo/cvm/setup.bash
2. Export ROS master
  $ export ROS_MASTER_URI=http://<VIPER_IP>:11311
3. Export host IP:
 $ export ROS_IP=<HOST_IP>

Steps to run the prebuilt FollowAruco sample app:
  $ cd installation_dir/bin
  $ ./cvm_follow_aruco_sample

Steps to build and run FollowAruco sample app:
1. Copy source code to a working dir
  $ mkdir ~/viper_tmp
  $ cd ~/viper_tmp
  $ cp -r installation_dir/share/code/ .
2. Build project
  $ mkdir build
  $ cd build
  $ ../cmake code/
  $ make
3. Run the app
  $ ./cvm_follow_aruco_sample

