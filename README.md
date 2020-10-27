# beginner_tutorials
ROS Publisher/Subscriber exmample


## Overview

ROS Tutorial to create and build a simple package with a publisher and subscriber.
Tutorial based on link (http://wiki.ros.org/ROS/Tutorials/WritingPublisherSubscriber%28c%2B%2B%29)


## Dependencies

The following dependencies are required to run this package:

1. ROS Melodic
2. catkin (http://wiki.ros.org/catkin#Installing_catkin)
3. Ubuntu 18.04 For installing ROS (http://wiki.ros.org/kinetic/Installation)
4. C++ 11

## Standard install via command-line
```
cd <<Your_catkin_workspace>>/src
mkdir beginner_tutorials
cd beginner_tutorials
git clone --recursive https://github.com/snehanyk05/beginner_tutorials
cd ../..
catkin_make
```

Open three terminals and run the following commands in them:

1. Terminal 1:
```
roscore
```

2. Terminal 2:
```
cd <<Your_catkin_workspace>>
source devel/setup.bash
rosrun beginner_tutorials talker
```

3. Terminal 3:
```
cd <<Your_catkin_workspace>>
source devel/setup.bash
rosrun beginner_tutorials listener
```

# TODO
[1] Navigating the ROS wiki - Complete
[2] Navigating the ROS Filesystem - Complete
[3] Creating a ROS Package - Complete
[4] Building a ROS Package - Complete
[5] Understanding ROS Nodes - Complete
[6] Understanding ROS Topics - Complete
[7] Writing a Simple Publisher and Subscriber - Complete
[8] Examining the Simple Publisher and Subscriber - Complete