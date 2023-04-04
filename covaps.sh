#!/bin/bash

#goal of this project is to setup the project 

echo "source ./install/setup.bash" >> ~/.bashrc

colcon build

source ~/.bashrc
