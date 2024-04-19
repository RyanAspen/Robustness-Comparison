# Install ORB-SLAM2
https://github.com/raulmur/ORB_SLAM2?tab=readme-ov-file

## C++11
For WSL, C++11 compiler is not by default installed. Run
```
sudo apt-get update
sudo apt-get install build-essential
```
and double check g++ version

## Pangolin v0.6
!! Use Pangolin v0.6 on 16.04 and check *Building* section
```
git clone https://github.com/stevenlovegrove/Pangolin.git --branch v0.6
```

## OpenCV 3.2.0
I found this useful
https://gist.github.com/umardx/dfce1dc74d925629a9eb8ad409740b47

There's an if-else bug in OpenCVCompilerOptions, comment out line 21 and 22
```
## Fix Flow control statements are not properly nested issue
sed -i '21,22 s/^/#/' cmake/OpenCVCompilerOptions.cmake
```

At the last step, may encounter symlink issue:
```
sudo chown $USER:$USER /usr/lib/wsl/lib # change permission
rm libcuda.so.1
rm libcuda.so
ln -s libcuda.so.1.1 libcuda.so.1 --force
ln -s libcuda.so.1.1 libcuda.so --force
```

check https://github.com/microsoft/WSL/issues/5548

## Eigen 3.2.10
Download, unzip, then 
```
mkdir build
cd build
cmake ..
make
sudo make install
```

## ORBSLAM

If accidentally installed 3.4.0, https://github.com/raulmur/ORB_SLAM2/issues/317
Refer to this
https://github.com/raulmur/ORB_SLAM2/pull/144 when error of usleep not defined

## ROS Kinetic
https://askubuntu.com/questions/1137931/unable-to-install-libopencv-nonfree-dev-in-ubuntu-16-04

## DSLAM
Make sure to setup github SSH before cloning `git@github.com:uzh-rpg/dslam_open.git`



