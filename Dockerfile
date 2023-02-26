FROM ubuntu:20.04
ARG DEBIAN_FRONTEND=noninteractive
WORKDIR /app
COPY . .
RUN apt-get update 
RUN apt-get install git cmake gcc g++ libeigen3-dev libtinyxml2-dev curl gnupg2 -y
RUN apt-get install liburdfdom-dev liburdfdom-headers-dev libconsole-bridge-dev -y
RUN git clone https://github.com/orocos/orocos_kinematics_dynamics.git
RUN mkdir -p orocos_kinematics_dynamics/orocos_kdl/build 
RUN cd orocos_kinematics_dynamics/orocos_kdl/build && cmake .. && make install 
RUN git clone https://github.com/ros/kdl_parser.git
RUN mkdir -p kdl_parser/kdl_parser/build 
RUN cd kdl_parser/kdl_parser/build && cmake .. && make && make install
RUN sh -c 'echo "deb http://packages.ros.org/ros/ubuntu focal main" > /etc/apt/sources.list.d/ros-latest.list'
RUN curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | apt-key add -
RUN apt-get update
RUN apt-get install ros-noetic-trac-ik -y
