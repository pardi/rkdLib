# RKDLib
RKDLib is a wrapper of the KDL library from Orocos, which uses Eigen and simplify the obtaining of kinematics and dynamics properties for any robot by URDF.


## Requirements: 
This package requires the installation of the following packages. To avoid issue with the lib versioning, please follow 
the order.

- cmake >= 3.4
- [kdl-orocos](https://www.orocos.org/kdl)
- [urdfdom](https://github.com/ros/urdfdom)
- [kdl_parser](https://github.com/ros/kdl_parser/tree/melodic-devel/kdl_parser)
- [trac-ik](https://bitbucket.org/traclabs/trac_ik/src/master/trac_ik_lib/)

## Installation
Clone the project repo:

    git clone git@gitlab.com:t.pardi/rkdlib.git 

Install:

    cd rkdlib/
    mkdir build
    cd build
    cmake ..

If you want to build the examples, use instead:
    
    cmake .. -DBUILD_EXAMPLES=ON

Then:

    make -j 4
    sudo make install


## Getting started

The class offers a simple interface within the namespace RKD.

Here, you can find a snippet of code for runninig your first example!

```C
#include <RKD/Robot.h>

using namespace RKD;

Robot myRobot;

```
