# tf2_manif
## A conversion package to convert [`tf2`](https://github.com/ros/geometry2) to/from [`manif`](https://github.com/artivis/manif) data structures.

<!-- [![Build Status](https://travis-ci.org/artivis/tf2_manif.svg?branch=master)](https://travis-ci.org/artivis/tf2_manif) -->
---

## Package Summary

-   Maintainer status: maintained
-   Maintainer: Jeremie Deray <deray.jeremie@gmail.com>
-   Author: Jeremie Deray <deray.jeremie@gmail.com>
-   License: MIT
-   Bug / feature tracker: https://github.com/artivis/tf2_manif/issues
-   Source: git https://github.com/artivis/tf2_manif.git (branch: master)

## Quick Start

<!--### Installation-->

<!--#### Binaries
```terminal
$ apt-get install ros-indigo-my-package
```-->
#### From source
```terminal
$ git clone https://github.com/artivis/tf2_manif.git
$ catkin build
```

## Some more details about the package

**For more information about the [`manif`](https://github.com/artivis/manif)
library please follow** [***this link***](https://github.com/artivis/manif).

#### Supported datatype:

| ROS message type | manif datatype | Transform function |
| :---       |   :---:   | :---: |
| [geometry_msgs::Vector3](http://docs.ros.org/melodic/api/geometry_msgs/html/msg/Vector3.html) | [Eigen::Vector3d](https://eigen.tuxfamily.org) | ```void doTransform(const Eigen::Vector3d& t_in, Eigen::Vector3d& t_out, const geometry_msgs::Transform& transform)``` |
| [geometry_msgs::Quaternion](http://docs.ros.org/melodic/api/geometry_msgs/html/msg/Quaternion.html) | [manif::SO3d](https://codedocs.xyz/artivis/manif/structmanif_1_1_s_o3.html) | ```void doTransform(const manif::SO3d& t_in, manif::SO3d& t_out, const geometry_msgs::Transform& transform)``` |
| [geometry_msgs::Pose](http://docs.ros.org/melodic/api/geometry_msgs/html/msg/Pose.html) | [manif::SE3d](https://codedocs.xyz/artivis/manif/structmanif_1_1_s_e3.html) | ```void doTransform(const manif::SE3d& t_in, manif::SE3d& t_out, const geometry_msgs::Transform& transform)``` |
| [geometry_msgs::Twist](http://docs.ros.org/melodic/api/geometry_msgs/html/msg/Twist.html) | [manif::SE3Tangentd](https://codedocs.xyz/artivis/manif/structmanif_1_1_s_e3_tangent.html) | Not available atm : [see here](http://wiki.ros.org/tf/Reviews/2010-03-12_API_Review) |>

<!--| [geometry_msgs::Vector3](http://docs.ros.org/melodic/api/geometry_msgs/html/msg/Vector3.html) | [Eigen::Vector3d](https://eigen.tuxfamily.org) | ```void doTransform(const Eigen::Vector3d& t_in, Eigen::Vector3d& t_out,```<br/>```const geometry_msgs::Transform& transform)``` |-->
