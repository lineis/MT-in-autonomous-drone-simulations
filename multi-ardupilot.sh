#!/bin/bash

gnome-terminal --tab -- bash -c "sim_vehicle.py -v ArduCopter -f gazebo-drone1 -I0 --out=tcpin:0.0.0.0:8000 " 
gnome-terminal --tab -- bash -c "sim_vehicle.py -v ArduCopter -f gazebo-drone2 -I1 --out=tcpin:0.0.0.0:8100 " 
 

