___
# Pera Swarm Active Localization and Navigation
___

## Overview

This research aims to develop a navigation and path-planning algorithm that integrates visual SLAM with reinforcement learning for a ground-aerial multi-vehicle system. Autonomous vehicles have become essential for extreme environments such as search and rescue, disaster relief, and infrastructure inspection, where human presence is limited. The combination of unmanned ground vehicles (UGVs) and unmanned aerial vehicles (UAVs) enhances mission efficiency by leveraging the complementary strengths of both systems—UGVs provide endurance and payload capacity, while UAVs offer aerial surveillance and rapid maneuverability.

## RL Module

open jupiter notebook through chrome
login to turing 
run jupyter nootebook list
that command show the threads and copy the token

on wsl terminal -
ssh -N -f -L localhost:8885:turing.ce.pdn.ac.lk:8892 e19423@tesla.ce.pdn.ac.lk

then search htttp://localhost:8885 browser

paste the token in the window then open the home 

 cd /Fyp/RL_Model

 run connection.ipynb

 it prints listining from port

run central sewer 

roslaunch turtlebot3_gazebo headless.launch world_name:=turtlebot3_world.world

lunch robo nodes 

roslaunch central_server robo_nodes.launch

run map merger 

rosrun central_server iter_mapp.py

run rviz for simlation

 rviz

run simulation

roslaunch turtlebot3_gazebo headless.launch world_name:=turtlebot3_world.world # selsct a world and repalce

data trafic forward to serwer through ssh

ssh -L 9002:localhost:9002 e19423@10.40.18.3

run the connection_py for each robot

rosrun central_server connect_tb3_0.py




