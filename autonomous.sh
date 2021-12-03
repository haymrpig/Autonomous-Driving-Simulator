#!/bin/bash
source ~/.bashrc

bash_file_name="/home/hong/simulator/project/devel/setup.bash"
launch_file_name="/home/hong/simulator/project/src/launch/skyautonet.launch"

source $bash_file_name

roslaunch $launch_file_name

