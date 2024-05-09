## Thesis - Metamorphic Testing in the context of Autonomous Drone Simulations

Metamorphic Testing has seen an immense increase in popularity among software engineers and scientists ever since it was first introduced in 1998. It is a software testing technique that focuses on relations between inputs and outputs of a program rather than the inputs and outputs themselves. These relations are referred to as Metamorphic Relations, whose construction is non-trivial and usually requires expertise and domain knowledge. The nature of this strategy facilitates the testing of programs even in the absence of a test oracle. Examples for such programs are search engines, compilers, or simulators, the latter of which will provide the basis for the implementation within this project.

In autonomous systems, it can be extremely difficult to decide whether the behaviour of some autonomous system meets its specifications, hence why applying Metamorphic Testing to this domain should show positive results. Autonomous drones, in particular, are becoming more and more popular for purposes like
surveillance or search operations, while software testing in this field is complex.

The goal of this project is to explore the applicability of Metamorphic Testing in autonomous systems by implementing the technique in respect of testing autonomous drones.


---

### Implementation

**Dependencies:**
- Gazebo
- Ardupilot Plugin
- SITL
- ROS
- DroneKit

*The final implementation uses the [GNC API](https://github.com/Intelligent-Quads/iq_gnc) where excellent online documentation for both installation and usage is available. The corresponding workspace that we used is /catkin_ws.*
  

**How to Launch**
```
Single Drone Environment:
terminal 1: roslaunch iq_sim file.launch
terminal 2: ./startsitl.sh (or run sim_vehicle.py -v ArduCopter -f gazebo-iris)
terminal 3: roslaunch iq_sim apm.launch (= MAVROS communication)
terminal 4: rosrun iq_gnc obs_avoid (obstacle avoidance script)

Multi-Drone Environment:
terminal 1: roslaunch iq_sim multi_drone.launch
terminal 2: ./multi-artdupilot.sh (= all SITL instances, i.e. for each drone: sim_vehicle.py -v ArduCopter -f gazebo-drone<int> -I<int>)
terminal 3: roslaunch iq_sim multi-apm.launch (= all MAVROS instances, i.e. for each drone: `roslaunch iq_sim apm.launch fcu_url:=udp://127.0.0.1:<port>@<port> mavros_ns:=/drone<int> tgt_system:=<int>` )
(...wait for ALL drones to use gps!)
terminal 4: roslaunch iq_gnc multi_obs_avoid.launch (starts all instances of the script)

Remember to check the launch files (which world/script is launched) as well as to use BASH (not zsh or others).
While Python script changes merely require restarting the script, changes to any C++ files will require to re-run the following:
catkin build
source ~/.bashrc
```
  

**Important How-To's:**
```
/* launch gazebo using ROS */
roscore&
source /opt/ros/noetic/setup.zsh  //unless added to bashrc
rosrun gazebo_ros gazebo <path>/worlds/example_world.world

/* run SITL */
cd <path>/ardupilot/ArduCopter
sim_vehicle.py -v ArduCopter -f gazebo-iris --console

/* optional: connect dronekit script to SITL port */
cd ardupilot_ws/src/test_pkg/src
python my_script.py --connect udpin:127.0.0.1:14550

/* optional: start monitoring (rviz etc.) */
rqt

```
  
  
**MAVProxy command examples:**
```
### manually send drone to point
mode guided
arm throttle
takeoff <altitude>
guided <x> <y> <altitude>
mode land // to land at curr pos ('mode' to list modes)
rtl       // return to launch point

# disarm before rebooting!

### make drone auto-move
wp load waypoints.txt
mode auto
...
mode land

### get data / output:
# graphs:
module load graph
graph <TAB>
# example: graph VFR_HUD.alt
# also: gtakeoff
```


### Documentation

- https://github.com/Intelligent-Quads/iq_gnc
- https://ardupilot.org/dev/docs/building-setup-linux.html#building-setup-linux
- https://ardupilot.org/dev/docs/setting-up-sitl-on-linux.html
- https://ardupilot.org/dev/docs/using-sitl-for-ardupilot-testing.html
- https://ardupilot.org/dev/docs/copter-sitl-mavproxy-tutorial.html
- http://gazebosim.org/tutorials?tut=ros_installing

