FROM ubuntu:20.04

# ROS 2 installation https://docs.ros.org/en/foxy/Installation/Ubuntu-Install-Debians.html
# Set locale
RUN apt update && \
	apt install locales

RUN locale-gen en_US en_US.UTF-8 \
	&& update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8

ENV LANG=en_US.UTF-8

# Setup sources
RUN apt install software-properties-common -y && \
	add-apt-repository universe

# Add ROS 2 GPG key
RUN apt update && apt install curl -y && \
	curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
RUN echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | tee /etc/apt/sources.list.d/ros2.list > /dev/null

# ROS-Base Install (Bare Bones): Communication libraries, message packages, 
# command line tools. No GUI tools.
# Development tools: Compilers and other tools to build ROS packages
RUN apt update && \
	apt upgrade -y && \
	apt install -y \
		ros-foxy-ros-base python3-argcomplete ros-dev-tools

WORKDIR /milrem