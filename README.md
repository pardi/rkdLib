[![License](https://img.shields.io/badge/License-BSD_3--Clause-blue.svg)](https://opensource.org/licenses/BSD-3-Clause)
![build](https://github.com/pardi/rkdLib/actions/workflows/docker-image.yml/badge.svg?event=push)
# RKDLib
RKDLib is a wrapper of the KDL library from Orocos, which uses Eigen and simplify the obtaining of kinematics and dynamics properties for any robot by URDF.


## Requirements: 
This package requires the installation of the following packages. To avoid issue with the lib versioning, please follow 
the order.

```
sudo apt update
sudo apt install -y cmake gcc g++ libeigen3-dev libtinyxml2-dev curl \
                    gnupg2 liburdfdom-dev liburdfdom-headers-dev \
                    libconsole-bridge-dev libboost-all-dev libnlopt-dev libnlopt-cxx-dev
```

- [kdl-orocos](https://github.com/orocos/orocos_kinematics_dynamics.git)
- [kdl_parser](https://github.com/pardi/kdl_parser.git)
- [trac-ik-lib-standalone](https://github.com/pardi/trac_ik_lib_standalone.git)

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

using namespace rkd;

Robot myRobot;

```

More examples can be found in [examples](examples)
