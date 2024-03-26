# Robustness-Comparison
Project Proposal - A Comparative Analysis on Redundancy and Robustness of Centralized and Decentralized Multi-Robot SLAM

# Swarm-SLAM installation guide

Reference from [Swarm-SLAM repo](https://github.com/MISTLab/Swarm-SLAM) and [startup guide](https://lajoiepy.github.io/cslam_documentation/html/md_startup-instructions.html)

## Install ROS2
I have Ubuntu 22.04 and ROS 2 Humble installed before. They used Foxy but Humble is also fine. Installation can be found [here](https://docs.ros.org/en/humble/Installation.html) andKindly check this youtube [link](https://www.youtube.com/watch?v=0aPbWsyENA8) or any other tutorial.

## Install repo packages
After cloning the Swarm-SLAM repo,
```
sudo apt install python3-vcstool
cd $SWARM_REPO
mkdir src
vcs import src < cslam.repos
```

## Install miniconda
Check this [link](https://docs.anaconda.com/free/miniconda/). For Linux, simply
```
mkdir -p ~/miniconda3
wget https://repo.anaconda.com/miniconda/Miniconda3-latest-Linux-x86_64.sh -O ~/miniconda3/miniconda.sh
bash ~/miniconda3/miniconda.sh -b -u -p ~/miniconda3
rm -rf ~/miniconda3/miniconda.sh
```
and initialize with
```
~/miniconda3/bin/conda init bash
~/miniconda3/bin/conda init zsh
```

## Create virtual env
```
conda create --name cslam
conda activate cslam
sudo apt install python3-pip 
```
Then
```
pip install -r requirements.txt # The requirements file is in the main Swarm-SLAM repo.You don't need torch if you are only using lidar.
```
If any of the packages are not installed, run
```
pip install $PACKAGES
or
conda install $PACKAGES
```
Make sure you are using the pip in the virtual env (not base)




