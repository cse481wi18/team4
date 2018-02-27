. /opt/ros/indigo/setup.bash
. ~/catkin_ws/devel/setup.bash

rm -rf labels
mkdir labels
rosrun perception extract_features Tennis_Ball.bag tennis
rosrun perception extract_features Expo.bag spray
rosrun perception extract_features Turtle.bag turtle
rosrun perception extract_features Yellow_Dog_Toy.bag yellow
mv *_label.bag labels
roslaunch applications lab34.launch data_dir:=/home/team4/catkin_ws/src/cse481wi18/stuff/labels/
