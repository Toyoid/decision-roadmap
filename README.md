# Decision Roadmap: A framework for learning high-level navigation decisions (or planning decisions) via deep reinforcement learning

## Basic Requirements

- Ubuntu 20.04
- ROS Noetic with some ROS package dependencies ([octomap](https://wiki.ros.org/octomap), [mavros](https://wiki.ros.org/mavros), [vision_msgs](https://wiki.ros.org/vision_msgs))
- Python 3.8.10 (with empy==3.3.4, rospkg)
- Pytorch 2.0.0 (with CUDA 11.8, tensorboard and wandb)

## Installation Guide

### 1. One-line Command installation of ROS

This repo requires to install ROS Noetic with Ubuntu 20.04. Assuming you have Ubuntu 20.04 already installed on your machine, you can install ROS Noetic with a super simple one-line command from [fishros](https://github.com/fishros/install). First, enter the following command in your Terminal:

```commandline
wget http://fishros.com/install -O fishros && . fishros
```

Then, type `1` to start the installation of ROS. Note that the messages for installation choices generated after this command are in Chinese. You may need a translator to follow the important messages.

### 2. Install ROS Package Dependencies

Once the ROS Noetic is properly setup, install the ROS packages ([octomap](https://wiki.ros.org/octomap), [mavros](https://wiki.ros.org/mavros), and [vision_msgs](https://wiki.ros.org/vision_msgs)) that this repo depends on:

```commandline
sudo apt install ros-${ROS_DISTRO}-octomap* && sudo apt install ros-${ROS_DISTRO}-mavros* && sudo apt install ros-${ROS_DISTRO}-vision-msgs
```

### 3. Setup Python Environment

We have provided the `requirements.txt` for the python environment of this repo.
To setup the python environment, first, create your virtual python environment with `python==3.8.10` (we recommend using conda to set up the environment).
Then Clone the repo:

```commandline
git clone https://github.com/Toyoid/decision-roadmap.git
```

And install the dependencies with:

```commandline
cd Path/To/decision-roadmap/
pip install -r requirements.txt
```

### 4. Build and Setup the Workspace

Lastly, setup environment variables for this repo. You need to properly set both the ROS environment variables after building the workspace and the python environment variable.
For the python environment variable, add `decision-roadmap` workspace to PYTHONPATH for package searching:

```
export PYTHONPATH="Path/To/decision-roadmap/ros/src/decision_roadmap_agent/scripts:$PYTHONPATH"
```

Then follow the standard catkin_make process to build the ROS workspace:

```commandline
cd Path/To/decision-roadmap/ros/
catkin_make
```

After that, setup environment variable by adding the following to your `~/.bashrc` file:

```commandline
source Path/To/decision-roadmap/ros/src/uav_simulator/gazeboSetup.bash
```

Optionally, we recommend that you also add the `decision-roadmap` workspace to your `~/.bashrc` file for the convenience of future usage:

```commandline
source Path/To/decision-roadmap/ros/devel/setup.bash
```

Now you are ready to run the project!

## Usage

Launch the robot system with topological graph: 

```
roslaunch autonomous_flight robot_system_gazebo.launch
```

Train the DRL planning agent: 

```
cd src/decision_roadmap_agent/scripts
python train/train_sac_drm_nav.py
```

Evaluate the DRL planning agent:

```
cd src/decision_roadmap_agent/scripts
python eval/eval_sac_drm_nav.py
```

## References

This repo is built upon the following works:

- [CERLAB-UAV-Autonomy](https://github.com/Zhefan-Xu/CERLAB-UAV-Autonomy)
- [Context_Aware_Navigation](https://github.com/marmotlab/Context_Aware_Navigation.git)
- [aerial_autonomy_development_environment](https://github.com/Bottle101/aerial_autonomy_development_environment.git)
