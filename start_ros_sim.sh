#!/bin/bash

# TODO cmdline args for tailing output

# if roscore is already running, skip starting it
if ! pgrep -x "roscore" > /dev/null
then
    echo "launching roscore..."
    roscore &
fi

# if gazebo is already running, skip starting it
if ! pgrep -x "roslaunch" > /dev/null
then
    echo "launching gazebo..."
    roslaunch fetch_gazebo playground.launch &
fi

catkin build && roslaunch web_teleop backend.launch

if ! pgrep -x "polymer serve" > /dev/null
then
    echo "Serving front-end..."
    polymer serve &
    firefox 'http://127.0.0.1:8081'
fi

