# RKDLib
RKDLib is a wrapper of the KDL library from Orocos, which uses Eigen and simplify the obtaining of kinematics and dynamics properties for any robot by URDF.

Dependencies:
- [kdl-orocos](https://www.orocos.org/kdl)
- [kdl_parser](https://github.com/ros/kdl_parser/tree/melodic-devel/kdl_parser)


## Usage

```C
#include <RKD/Robot.h>

using namespace RKD;

Robot myRobot;

```
