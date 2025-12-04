# SignLoc: Robust Localization using Navigation Signs and Public Maps

This repository contains the implementation of the following [publication](https://arxiv.org/pdf/2508.18606):
```bibtex
@article{zimmerman2025ral,
  title={SignLoc: Robust Localization using Navigation Signs and Public Maps},
  author={Zimmerman, Nicky and Loo, Joel and Agrawal, Ayush and Hsu, David},
  journal={IEEE Robotics and Automation Letters},
  year={2025},
  publisher={IEEE}
}
```

## Results
Our live demo for both localization on the Spot platform can seen in the following video:
[![](http://img.youtube.com/vi/EgpQHuaMOrc/0.jpg)](https://www.youtube.com/watch?v=EgpQHuaMOrc "SIGNLOC")


## Abstract
Navigation signs and maps, such as floor plans and street maps, are widely available and serve as ubiquitous aids for way-finding in human environments. Yet, they are rarely used by robot systems. This paper presents SignLoc, a global localization method that leverages navigation signs to localize the robot on publicly available maps—specifically floor plans and OpenStreetMap (OSM) graphs–without prior sensor-based mapping. SignLoc first extracts a navigation graph from the input map. It then employs a probabilistic observation model to match directional and locational cues from the detected signs to the graph, enabling robust topo-semantic localization within a Monte Carlo framework. We evaluated SignLoc in diverse large-scale environments: part of a university campus, a shopping mall, and a hospital complex. Experimental results show that SignLoc reliably localizes the robot after observing only one to two signs.

![Motivation](pics/Motivation.jpg)

## Installation

We provide both ROS Noetic and ROS Humble dockers for the installation

## Execution

For ROS Noetic, run
```
roslaunch signloc_ros mclcpp.launch
```
We also have a rosbag to try out with the code, but because our university has some issues with publicly shared storage, we can't just link it here (for now).
So if you are interested, email us, and we can share the file with you by adding your email to "can view" list.
You can also try this [dropbox link](https://www.dropbox.com/scl/fi/sjz2os81lg7k8sb7iqqhp/ral2025.bag?rlkey=d2xizcrpulclp7lv25djz57ga&e=1&st=bdkzr1et&dl=0), maybe it actually works.
