# pygazebo_ros

[![Build Status](https://travis-ci.com/qgallouedec/pygazebo_ros.svg?branch=devel)](https://travis-ci.com/qgallouedec/pygazebo_ros)
                 
## Installation

Install `gazebo-ros` package

```bash
sudo apt install ros-melodic-gazebo-ros
```

Clone de repository and install it.

```bash
git clone https://github.com/qgallouedec/pygazebo_ros
pip install pygazebo_ros
```

## Usage

Make sure that a roscore and gazebo_ros are running 

Then, run this simple example:

```python
import pygazebo_ros

my_gazebo = pygazebo_ros.GazeboROS()
my_gazebo.spawn_cube()
```
