-c /bin/bash

apt-get update 
DEBIAN_FRONTEND=noninteractive apt-get install -y keyboard-configuration
DEBIAN_FRONTEND=noninteractive TZ="Europe/London" apt-get install -y tzdata
apt-get -y upgrade

apt-get install -y --force-yes \
    wget \
    vim-gtk \
    zip \
    unzip \
    git \
    build-essential \
    software-properties-common \
    xterm \
    pypy \
    cmake \
    curl \
    python \
    psmisc \
    apt-utils \
    python3-pip \
    mesa-utils \
    software-properties-common \
    htop \
    nvtop \
    pciutils

add-apt-repository ppa:ubuntu-toolchain-r/test -y
apt-get update
apt-get install gcc-11 g++-11 -y
update-alternatives --install /usr/bin/gcc gcc /usr/bin/gcc-11 60 --slave /usr/bin/g++ g++ /usr/bin/g++-11
update-alternatives --config gcc

apt install zstd

# Installation of ROS Noetic and essential packages
sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | apt-key add -
apt update
apt install -y --force-yes \
    ros-noetic-desktop \
    ros-noetic-rosbash \
    python3-rosdep \
    python3-wstool \
    python3-catkin-tools

# Init rosdep and update
rosdep init
rosdep update

apt-get update

# Installation of MoveIt! related packages
apt install -y --force-yes \
    ros-noetic-libfranka \
    ros-noetic-franka-ros \
    ros-noetic-panda-moveit-config \
    ros-noetic-effort-controllers \
    ros-noetic-position-controllers \
    ros-noetic-ros-controllers \
    ros-noetic-effort-controllers \
    ros-noetic-rqt-controller-manager \
    ros-noetic-four-wheel-steering-msgs \
    ros-noetic-urdf-geometry-parser \
    ros-noetic-urdf-geometry-parser \
    ros-noetic-rosparam-shortcuts \
    ros-noetic-joy \
    ros-noetic-moveit-resources-panda-moveit-config \
    ros-noetic-vision-msgs \
    ros-noetic-depth-image-proc

# Let's have a custom PS1 to help people realise in which container they are
# working.
CUSTOM_ENV=/.singularity.d/env/99-zz_custom_env.sh
cat >$CUSTOM_ENV <<EOF
#!/bin/bash
PS1="[PBPF] Singularity> \w \$ "
EOF
chmod 755 $CUSTOM_ENV
