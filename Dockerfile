FROM ubuntu:20.04 AS ci_minimal
ARG DEBIAN_FRONTEND=noninteractive
WORKDIR /app

RUN apt-get update 
RUN apt-get install -y git cmake gcc g++ libeigen3-dev libtinyxml2-dev curl gnupg2 liburdfdom-dev liburdfdom-headers-dev libconsole-bridge-dev libboost-all-dev libnlopt-dev libnlopt-cxx-dev

RUN mkdir setup_dep && cd setup_dep && git clone https://github.com/orocos/orocos_kinematics_dynamics.git
RUN mkdir -p setup_dep/orocos_kinematics_dynamics/orocos_kdl/build 
RUN cd setup_dep/orocos_kinematics_dynamics/orocos_kdl/build && cmake .. && make install 

RUN cd setup_dep/ && git clone https://github.com/pardi/kdl_parser.git
RUN mkdir -p setup_dep/kdl_parser/kdl_parser/build && cd setup_dep/kdl_parser/kdl_parser/build && cmake .. && make && make install

RUN cd setup_dep/ && git clone https://github.com/pardi/trac_ik_lib_standalone.git
RUN mkdir -p setup_dep/trac_ik_lib_standalone/build && cd setup_dep/trac_ik_lib_standalone/build && cmake .. && make && make install


COPY . /app

FROM ci_minimal AS build_tests
RUN git clone --recurse-submodules https://github.com/pardi/rkdLib.git
RUN mkdir -p rkdLib/build && cd rkdLib/build && cmake .. && make && ctest
