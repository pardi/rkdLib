# RKDLib
RKDLib is a wrapper of the KDL library from Orocos, which uses Eigen and simplify the obtaining of kinematics and dynamics properties for any robot by URDF.


Dependencies: (Please follow the order)
- [kdl-orocos](https://www.orocos.org/kdl)
- [urdfdom](https://github.com/ros/urdfdom)
- [kdl_parser](https://github.com/ros/kdl_parser/tree/melodic-devel/kdl_parser)
- [trac-ik](https://bitbucket.org/traclabs/trac_ik/src/master/trac_ik_lib/)


## Usage

```C
#include <RKD/Robot.h>

using namespace RKD;

Robot myRobot;

```
