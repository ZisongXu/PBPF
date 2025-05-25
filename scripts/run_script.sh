#!/bin/bash
source /opt/ros/noetic/setup.bash

if [ ! -d "$HOME/catkin_ws/devel" ]; then
    cd $HOME/catkin_ws/
    catkin_make
    pip install pyquaternion==0.9.9
    pip3 install pybullet==3.2.7
    pip3 install pandas==2.0.0
    pip3 install scikit-surgerycore==0.8.0
    pip3 install seaborn==0.12.2
    pip3 install matplotlib==3.7.1
    pip install --upgrade pip
fi

source $HOME/catkin_ws/devel/setup.bash
source $HOME/.bashrc
alias xterm="xterm -fa 'Monospace' -fs 14 &"

