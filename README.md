# beginner_tutorials
ROS Publisher/Subscriber example


## Overview

ROS Tutorial to create and build a simple package with a publisher and subscriber.
Tutorial based on link (http://wiki.ros.org/ROS/Tutorials/WritingPublisherSubscriber%28c%2B%2B%29)
We also create a service in the talker file so that we can send custom talker messages. 
We also test out ros' 5 logger levels and also check the ros console's behaivor.

## License
```
MIT License

Copyright (c) 2020 Sneha Nayak

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
```

## Dependencies

The following dependencies are required to run this package:

1. ROS Melodic
2. catkin (http://wiki.ros.org/catkin#Installing_catkin)
3. Ubuntu 18.04 For installing ROS (http://wiki.ros.org/melodic/Installation)
4. C++ 11
5. catkin

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

We can now pass logger parameters through the commandline as follow:

```
cd <<Your_catkin_workspace>>
source devel/setup.bash
roslaunch beginner_tutorials beginner_tutorials.launch param:=error
```
param can have any of these logger level values: error, warn, debug, info, fatal



# TODO
[1] Navigating the ROS wiki - Complete 

[2] Navigating the ROS Filesystem - Complete

[3] Creating a ROS Package - Complete

[4] Building a ROS Package - Complete

[5] Understanding ROS Nodes - Complete

[6] Understanding ROS Topics - Complete

[7] Writing a Simple Publisher and Subscriber - Complete

[8] Examining the Simple Publisher and Subscriber - Complete

[9] Created branch Week9_HW 

[10] Getting started with roswtf - Complete

[11] Understanding ROS Services and Parameters - Complete

[12] Using rqt_console and roslaunch - Complete
