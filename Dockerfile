FROM ubuntu:20.04 AS ci_minimal
ARG DEBIAN_FRONTEND=noninteractive
WORKDIR /app
COPY . .
RUN apt-get update 
RUN apt-get install git cmake gcc g++ libeigen3-dev libtinyxml2-dev curl gnupg2 -y
RUN apt-get install liburdfdom-dev liburdfdom-headers-dev libconsole-bridge-dev -y


FROM ci_minimal AS ci_orocos
RUN mkdir setup_dep && cd setup_dep && git clone https://github.com/orocos/orocos_kinematics_dynamics.git
RUN mkdir -p setup_dep/orocos_kinematics_dynamics/orocos_kdl/build 
RUN cd setup_dep/orocos_kinematics_dynamics/orocos_kdl/build && cmake .. && make install 

FROM ci_orocos AS ci_kdl_parser
RUN cd setup_dep/ && git clone https://github.com/ros/kdl_parser.git
RUN mkdir -p setup_dep/kdl_parser/kdl_parser/build && cd setup_dep/kdl_parser/kdl_parser/build && sed 'x; ${s/.*/          DESTINATION include\/${PROJECT_NAME}\/)/;p;x}; 1d' ../CMakeLists.txt > CMakeLists.txt && mv CMakeLists.txt ../CMakeLists.txt && cmake .. && make && make install


FROM ci_kdl_parser AS ci
RUN sh -c 'echo "deb http://packages.ros.org/ros/ubuntu focal main" > /etc/apt/sources.list.d/ros-latest.list'
RUN curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | apt-key add -
RUN apt-get update
RUN apt-get install ros-noetic-trac-ik ros-noetic-nlopt -y


FROM ci AS build_tests
RUN mkdir build && cd build && cmake .. && make && ctest
