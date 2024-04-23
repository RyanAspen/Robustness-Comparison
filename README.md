# A Comparative Analysis on Robustness of Centralized Multi-Robot SLAM
In this project we tested robustness between two ORB-SLAM based centralized multi-robot SLAM methods, [JORB-SLAM](https://github.com/um-mobrob-t12-w19/JORB-SLAM/) and [Collaborative ORB-SLAM2](https://github.com/d-vo/collab_orb_slam2), along with the performance of single agent [ORB_SLAM2](https://github.com/raulmur/ORB_SLAM2) on the same KITTI dataset.

## Dataset
Download the KITTI odometry dataset (grayscale images) from [here](http://www.cvlibs.net/datasets/kitti/eval_odometry.php)
We will only be using Sequence 00, but the code can easily be extended to work on the other sequences as well.

# 1. ORB_SLAM2
**Authors:** [Raul Mur-Artal](http://webdiis.unizar.es/~raulmur/), [Juan D. Tardos](http://webdiis.unizar.es/~jdtardos/), [J. M. M. Montiel](http://webdiis.unizar.es/~josemari/) and [Dorian Galvez-Lopez](http://doriangalvez.com/) ([DBoW2](https://github.com/dorian3d/DBoW2))

ORB-SLAM2 is a real-time SLAM library for **Monocular**, **Stereo** and **RGB-D** cameras that computes the camera trajectory and a sparse 3D reconstruction (in the stereo and RGB-D case with true scale). It is able to detect loops and relocalize the camera in real time.

Both centralized multi-robot SLAM we tested are ORB_SLAM2 based. Kindly refer to [here for original implementation.](https://github.com/raulmur/ORB_SLAM2) We tested on Ubuntu 16.04

After installing all required dependencies, clone the `ORB_SLAM2` folder and make some configurations. Open "Examples/Stereo/stereo_kitti.cc". The constants defined at lines 67-69 can all be adjusted.
- `fail_rate` refers to the fraction of frames to randomly remove from.
- `seq_start` and `seq_end` are the starting and ending frame index.

To generate matched feature map points, follow
```
cd single_orb_slam2
chmod +x build.sh
./build.sh
./Examples/Stereo/stereo_kitti Vocabulary/ORBvoc.txt Examples/Stereo/KITTIX.yaml PATH_TO_DATASET_FOLDER/dataset/sequences/SEQUENCE_NUMBER
```
To compare between two generated map point file. simply configure the paths and run `/results/mapAMDcalc.cpp`

# 2. JORB-SLAM
**Authors:** Martin D Deegan, Yuanxin Zhong, Purva Kulkarni, Christine Searle, Kaustav Chakraborty.


To install JORB-SLAM, use the following instructions. We tested with Ubuntu 18.04.
```
cd JORB_SLAM
./install_apriltags.sh
./install_pangolin.sh
chmod +x build.sh
./build.sh
```

**NOTE** For some setups, running the above commands may result in cc1plus errors due to memory constraints. To get around this, run each command in the scripts one at a time, replacing make j8 commands with regular make commands (for example, ```make -j$((num_procs_avail > 1 ? num_procs_avail : 1))``` becomes ```make```). This will result in using a single-processor for building.

**NOTE** There is a known bug in how the Eigen library is installed which results in not being able to build. Make sure to remove the file "cmake_modules/FindEigen3.cmake" and edit line 50 in "CMakeLists.txt" to replace "EIGEN3_INCLUDE_DIR" with the path to your Eigen installation. In my case, this was "/usr/lib/Eigen".

To generate a map point cloud, open "Examples/Stereo/stereo_kitti.cc". The constants defined at lines 36-43 can all be adjusted. 
- seq_len must be set to the index of the last frame in the sequence being processed.
- seq_A_start, seq_B_start, and seq_C_start refer to the first indices for Robots A, B, and C to process
- seq_A_end, seq_B_end, and seq_C_end refer to the last indices for Robots A, B, and C to process
- fraction_to_remove refers to the fraction of frames to randomly remove from

When these constants are set to what you want, run the following:
```
cd build
make stereo_kitti
cd ..
./Examples/Stereo/stereo_kitti Vocabulary/ORBvoc.txt Examples/Stereo/KITTI00-02.yaml {PATH_TO_KITTI_DATASET}/dataset/sequences/00 Examples/KittiServerConfig.yaml
```
This will generate a file called "mapOutput.txt" that contains a pointcloud. 

# 3. Collab_orb_slam2

**Authors:**
Dominik Van Opdenbosch  and Eckehard Steinach
Chair of Media Technology, Technical University of Munich. Reference from [collab_orb_slam2](https://github.com/d-vo/collab_orb_slam2). We tested on Ubuntu 18.04 and ROS Melodic.

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
### Stereo
Start Server in new window
```
./Examples/ROS/compression/KittiServerStereo -v Vocabulary/voc_k10_l_5_N_100000.txt -c Vocabulary/stats_8b.vstats -s Examples/ROS/compression/
```
Start Agent, change PATH_TO_DATASET_FOLDER with your data folder, SEQUENCE_NUMBER with KITTI sequence number like 00/07 etc. AGEND_ID with 1/2/etc
```
./Examples/ROS/compression/KittiAgentStereo -v Vocabulary/voc_k10_l_5_N_100000.txt -c Vocabulary/stats_8b.vstats -i PATH_TO_DATASET_FOLDER/dataset/sequences/SEQUENCE_NUMBER -r AGEND_ID -s  Examples/ROS/compression/KITTIX.yaml
```
Remember to start server first before starting the agent

## Generating Pointcloud Statistics

Once pointcloud files are generated, comparison statistics can be calculated with "mapAMDcalc.cpp". Adjust the files paths at lines 25 and 53 to be the paths to two different pointcloud files. Once that is done, run the following:
```
g++ mapAMDcalc.cpp
./a.out
```
