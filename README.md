# Robustness-Comparison
Project Proposal - A Comparative Analysis on Redundancy and Robustness of Centralized and Decentralized Multi-Robot SLAM

# JORB-SLAM

To install JORB-SLAM, use the following instructions. We tested with Ubuntu 18.04.
git clone https://github.com/um-mobrob-t12-w19/ORB_SLAM2 ORB_SLAM2
cd ORB_SLAM2
./install_apriltags.sh
./install_pangolin.sh
chmod +x build.sh
./build.sh

**NOTE** For some setups, running the above commands may result in cc1plus errors due to memory constraints. To get around this, run each command in the scripts one at a time, replacing make j8 commands with regular make commands (for example, "make -j$((num_procs_avail > 1 ? num_procs_avail : 1))" becomes "make"). This will result in using a single-processor for building.

**NOTE** There is a known bug in how the Eigen library is installed which results in not being able to build. Make sure to remove the file "cmake_modules/FindEigen3.cmake" and edit line 50 in "CMakeLists.txt" to replace "EIGEN3_INCLUDE_DIR" with the path to your Eigen installation. In my case, this was "/usr/lib/Eigen".

To generate a map point cloud, open "Examples/Stereo/stereo_kitti.cc". The constants defined at lines 36-43 can all be adjusted. 
- seq_len must be set to the index of the last frame in the sequence being processed.
- seq_A_start, seq_B_start, and seq_C_start refer to the first indices for Robots A, B, and C to process
- seq_A_end, seq_B_end, and seq_C_end refer to the last indices for Robots A, B, and C to process
- fraction_to_remove refers to the fraction of frames to randomly remove from

When these constants are set to what you want, run the following:
cd build
make stereo_kitti
cd ..
./Examples/Stereo/stereo_kitti Vocabulary/ORBvoc.txt Examples/Stereo/KITTI00-02.yaml {PATH_TO_KITTI_DATASET}/dataset/sequences/00 Examples/KittiServerConfig.yaml
 
# Collab_orb_slam2

Reference from [collab_orb_slam2](https://github.com/d-vo/collab_orb_slam2). I tested on Ubuntu 18.04 and ROS Melodic

## Building
Install ROS Melodic
```
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
sudo apt install curl
curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -
sudo apt update
sudo apt install ros-melodic-desktop-full
echo "source /opt/ros/melodic/setup.bash" >> ~/.bashrc
source ~/.bashrc
source /opt/ros/melodic/setup.bash
sudo apt install python-rosdep python-rosinstall python-rosinstall-generator python-wstool build-essential
sudo apt install python-rosdep
sudo rosdep init
rosdep update
```
```
sudo apt-get install libglew-dev libboost-all-dev libopencv-dev build-essential cmake cmake-gui libeigen3-dev
git clone https://github.com/d-vo/collab_orb_slam2
cd collab_orb_slam2
chmod +x build.sh
./build.sh
```


If you want to use ROS to run the compression make sure to have ROS installed and added the Examples/ROS path to your ROS_PACKAGE_PATH: (Make sure to chnage "YOUR_PATH_TO" with your folder)
```
export ROS_PACKAGE_PATH=${ROS_PACKAGE_PATH}:YOUR_PATH_TO/collab_orb_slam2/Examples/ROS
echo 'export ROS_PACKAGE_PATH=${ROS_PACKAGE_PATH}:YOUR_PATH_TO/collab_orb_slam2/Examples/ROS' >> ~/.bashrc 
source ~/.bashrc

chmod +x build_ros_compress.sh
./build_ros_compress.sh
```
If you get eigen3 error while installation use this 
```
sudo ln -s /usr/include/eigen3/Eigen /usr/include/Eigen
```

## Dataset
Download the KITTI odometry dataset (grayscale images) from [here](http://www.cvlibs.net/datasets/kitti/eval_odometry.php)
Two simple examples of how to run the feature compression and collaborative mapping using 1) depth coding and 2) stereo coding. Every agent has a unique identifier. The bitstream will be published as /featComp/bitstreamN, where N is the agent id. Please note that the configuration for the server is fixed in the source code for agent 0 running KITTI 00 and agent 1 running KITTI 07
```
roscore
```
### Depth
Start Server in new window
```
./Examples/ROS/compression/KittiServerDepth -v Vocabulary/voc_k10_l_5_N_100000.txt -c Vocabulary/stats_8b.vstats -s Examples/ROS/compression/
```
Start Agent, change PATH_TO_DATASET_FOLDER with your data folder, SEQUENCE_NUMBER with KITTI sequence number like 00/07 etc. AGEND_ID with 1/2/etc
```
./Examples/ROS/compression/KittiAgentDepth -v Vocabulary/voc_k10_l_5_N_100000.txt -c Vocabulary/stats_8b.vstats -i PATH_TO_DATASET_FOLDER/dataset/sequences/SEQUENCE_NUMBER -r AGEND_ID -s  Examples/ROS/compression/KITTIX.yaml
```
### stereo
Start Server in new window
```
./Examples/ROS/compression/KittiServerStereo -v Vocabulary/voc_k10_l_5_N_100000.txt -c Vocabulary/stats_8b.vstats -s Examples/ROS/compression/
```
Start Agent, change PATH_TO_DATASET_FOLDER with your data folder, SEQUENCE_NUMBER with KITTI sequence number like 00/07 etc. AGEND_ID with 1/2/etc
```
./Examples/ROS/compression/KittiAgentStereo -v Vocabulary/voc_k10_l_5_N_100000.txt -c Vocabulary/stats_8b.vstats -i PATH_TO_DATASET_FOLDER/dataset/sequences/SEQUENCE_NUMBER -r AGEND_ID -s  Examples/ROS/compression/KITTIX.yaml
```
Remember to start server first before starting the agent


