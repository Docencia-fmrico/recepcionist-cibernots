# Seek_and_capture_cibernots

<h3 align="center">Seek and Capture </h3>

<div align="center">
<img width=100px src="https://img.shields.io/badge/status-finished-brightgreen" alt="explode"></a>
<img width=100px src="https://img.shields.io/badge/license-Apache-orange" alt="explode"></a>
</div>


## Table of Contents
- [Table of Contents](#table-of-contents)
- [Dependencies](#Dependencies)
- [How to execute the programs](#How-to-execute-the-programs)
- [Tf explanation](#Tf-explanation)
- [Behavior tree](#Behavior-tree)
- [DialogFlow explanation](#DialogFlow-explanation)
- [BT NODES](#BT-NODES)
- [license](#license)
- [Video_demostration](#Video_demostration)

## Dependencies

To use this program, you will need to have the following packages installed:

- ROS2
- Darknet ROS (version 1.2 or later): This package provides a pre-trained convolutional neural network (CNN) for person detection using a camera.
- Groot: This is a graphical user interface (GUI) program that will allow you to manually control the robot. It is optional, but highly recommended.
- BehaviorTree.CPP: for the robot's actuaction we will use behaviour trees, by accessing the following link, you can clone the repository and follow the compilation steps: https://github.com/facontidavide/BehaviorTree.CPP
- ZMQLIB: as the behavior tree is external to ros, it needs an IOT communication middleware for the communication between nodes, this is why we use ZMQ.
- DialogFlow: GB-dialog contains the library DialogInterface from which we will inherit to develop our dialogue actions. Each action would be specific to an intent (Dialogflow concepts).

You can install Darknet ROS by following the instructions in its GitHub repository:

Snippet (install Darknetros):
``` bash
cd ~/ros2_ws/src
git clone --recursive https://github.com/leggedrobotics/darknet_ros.git
cd darknet_ros
git checkout ros2
cd ../..
rosdep install --from-paths src --ignore-src --rosdistro foxy -y
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release

```
You can also install Groot following the nexts steps:
Snippet (install Groot):
``` bash
cd ~/ros2_ws/src

colcon build

```

You can  install ZMQ following the nexts steps:
Snippet (install ZMQ):
``` bash
# search libzmq
apt-cache search zmq

# install
sudo apt-get install libzmq3-dev libboost-dev

```

You can install install DialogFlow following the nexts steps:
``` bash
#If you don't have vcs tool, install it with:
sudo apt-get install python3-vcstool wget libgst-dev libgst7 libgstreamer1.0-* libgstreamer-plugins-base1.0-dev

cd <workspace>/src
mkdir dialog
cd dialog
wget https://raw.githubusercontent.com/IntelligentRoboticsLabs/gb_dialog/ros2/gb_dialog.repos
vcs import < gb_dialog.repos

cd <workspace>
rosdep install --from-paths src --ignore-src -r -y

#install requirements:
sudo apt-get install portaudio19-dev
cd <workspace>/src/dialog/dialogflow_ros2
pip3 install -r requirements.txt
```

Finally, follow this steps:
Google Cloud and DialogFlow Setup

    Go to Google Cloud Console.
    Create a new project.
    Go to the Kick Setup.
    Enable API.
    Create Service Account.
    Click on your service account. Create key & download the JSON File. Rename and move it t your HOME as ~/df_api.json.
    Go to DialogFlow Console.
    Create new Agent & select the project.
    Edit dialogflow_ros/config/param.yaml and write down your project id. You can find it in the DialogFlow Console, clicking in the gear icon.
    Add export ```bash GOOGLE_APPLICATION_CREDENTIALS='/home/<user>/df_api.json'`` to your .bashrc and change user.
    
    
## How to execute the programs

First connect the base and the camera then :
Snippet (launch base):
``` bash
ros2 launch ir_robots kobuki.launch.py # Driver of the kobuki
```
-----------------------------------------------------------------------

-----------------------------------------------------------------------
Snippet (launch program):
``` bash


```
-----------------------------------------------------------------------

## Tf explanation


### Transforms
<div align="center">
<img width=600px src="https://user-images.githubusercontent.com/90764494/228062043-d54209aa-46d8-424b-801e-4d76e9d219b2.png?raw=true" alt="explode"></a>
</div>

### Computer Graph
<div align="center">
<img width=600px src="https://user-images.githubusercontent.com/90764494/228062725-21386615-7479-492d-ac1e-7fba662bd6a2.png?raw=true" alt="explode"></a>
</div>

## Behavior tree
Remember that the most basic operation is a tick (a function call) that propagates to the children and returns 3 possibilities: SUCCES, FAILURE AND RUNNING.


## BT NODES




## Implements

In addition to all the above-mentioned implementations, we created some more that we did not have time to test.


## Video_demostration

## license 
<a rel="license" href="https://www.apache.org/licenses/LICENSE-2.0"><img alt="Apache License" style="border-width:0" src="https://www.apache.org/img/asf-estd-1999-logo.jpg" /></a><br/>(Cibernots) </a><br/>This work is licensed under a <a rel="license" href="https://www.apache.org/licenses/LICENSE-2.0">Apache license 2.0
