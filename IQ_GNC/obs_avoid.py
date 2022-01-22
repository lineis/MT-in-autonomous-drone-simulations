#! /usr/bin/env python

"""
Project for Bachelor Thesis:
Metamorphic Testing in the context of Autonomous Drone Simulations
Linus Eisele

Taken and/or adapted from:
https://dronekit-python.readthedocs.io/en/latest/examples/index.html © Copyright 2015-2016, 3D Robotics
https://github.com/Intelligent-Quads/iq_gnc MIT License © Copyright 2020 Intelligent-Quads
"""

############################################################ IMPORTS ############################################################

# imports for iq_gnc API
import rospy
from sensor_msgs.msg import LaserScan
from iq_gnc.py_gnc_functions import *
from math import cos, sin, pow, radians, sqrt
from geometry_msgs.msg import Point

# imports for dronekit
import argparse
from dronekit import connect, VehicleMode
from dronekit import LocationGlobalRelative
import dronekit_sitl

import time
import math
import numpy as np
import threading

# import auxiliary funcionts
from mt_obs_avoid_functions import *


############################################################ CONSTANTS ############################################################

DRONE_NO = 1 # for multiple drones, duplicate program and increment this number (filename equal to that in multi_obs_avoid.launch)
MAX_DETECTION_RANGE = 10
MIN_DETECTION_RANGE = 0.35
VELOCITY = 1            
TAKEOFF_HEIGHT = 2  
WAYPOINT_REACHED_TOL = 0.5  
DISTANCE_MEASURE_FREQ = 1
AVOIDANCE_MONITOR_FREQ = 1
DELTA_T = 8
DELTA_D = 1


############################################################ CONNECTION ############################################################

def start_sitl_and_connect_to_vehicle(connection_string):
    """
    Functionality to start SITL and automatically connect to vehicle
    (Alternatively, can run separately from <path>/ardupilot/ArduCopter: sim_vehicle.py -v ArduCopter -f gazebo-iris --console 
    and manually obtain the port from the console output, i.e. udpin:127.0.0.1:14550)
    """
    if not connection_string:
        # Start SITL 
        sitl = dronekit_sitl.start_default()
        connection_string = sitl.connection_string()

    # Connect to the Vehicle
    print(f"Connecting to drone {DRONE_NO} on {connection_string}")
    vehicle = connect(connection_string, wait_ready=True)

    return vehicle



############################################################ INIT ############################################################

# TODO: use __init__ class and clean up in object oriented manner

# instantiate API object as global var
drone = gnc_api()

# instantiate global vehicle object for drone data
if DRONE_NO == 1:
    vehicle = start_sitl_and_connect_to_vehicle("udpin:127.0.0.1:14550")
elif DRONE_NO == 2:
    vehicle = start_sitl_and_connect_to_vehicle("udpin:127.0.0.1:14560")

# global var to count the number of avoidance maneuvers
avoidance_maneuver_counter = 0

# global sum to count travelled distance
travel_distance = 0

# global vars to monitor avoidance
prevAvoided = 0
currAvoided = 0




############################################################ MONITORING FUNCTIONS ############################################################

def get_distance_travelled():
    global travel_distance
    """
    Continuously adds up the distance travelled.
    (No built-in functionality or rostopic was found for this cause)
    """
    while True:
        oldPos = vehicle.location.global_frame
        time.sleep(DISTANCE_MEASURE_FREQ)
        newPos = vehicle.location.global_frame
        travel_distance += abs(get_distance_metres(oldPos, newPos))
        #rospy.loginfo("\nTRAVELLED DISTANCE = %dm\n", travel_distance)


def laser_cb(msg):
    global currAvoided
    """
    Performs object detection and avoidance according to a paper by Maria Isabel Ribeiro
    (translated from c++ by Sahas Ananth). This is the callback function of a subscriber node
    to a ROS topic, hence is invoked whenever messages are received, with the message as
    argument.
    """
    # parse the lidar data
    cr_scan = LaserScan()
    cr_scan = msg
    avoid_x = 0.0
    avoid_y = 0.0
    avoid = False

    for i in range(1, len(cr_scan.ranges)):
        # distances in meters from which avoidance is triggered
        d0 = MAX_DETECTION_RANGE
        k = 0.5 # was 0.1 for most testing
        
        if cr_scan.ranges[i] < d0 and cr_scan.ranges[i] > MIN_DETECTION_RANGE:
            avoid = True
            currAvoided += 1 # increment for monitoring
            x = cos(cr_scan.angle_increment * i)
            y = sin(cr_scan.angle_increment * i)
            u = (-0.5 * k * pow(((1/cr_scan.ranges[i]) - (1/d0)), 2.0))

            # the final vectors are the sum of all the potentials in our field
            avoid_x += (x*u)
            avoid_y += (y*u)

    # rotate vectors according to the correct heading
    cr_heading = radians(drone.get_current_heading())
    avoid_x = (avoid_x * cos(cr_heading)) - (avoid_y * sin(cr_heading))
    avoid_y = (avoid_x * sin(cr_heading)) + (avoid_y * cos(cr_heading))

    if avoid:
        dist = sqrt(pow(avoid_x, 2) + pow(avoid_y, 2))

        # normalize the vector in case its magnitude exceeds 3 meters
        if dist > 3:
            avoid_x = (3 * (avoid_x/dist))
            avoid_y = (3 * (avoid_y/dist))
        
        # get the current location from the drone
        cur_pose = Point()
        cur_pose = drone.get_current_location()
        # send the new destination
        drone.set_destination(avoid_x + cur_pose.x,
                              avoid_y + cur_pose.y,
                              2, 0)

def avoidance_detected():
    """
    Investigates parameters and returns whether an avoidance maneuver has occurred.
    """
    global prevAvoided
    prevAvoided = currAvoided

    # if drone slows down heavily, check whether a significant change of location follows within x seconds
    if vehicle.velocity[0] < 0.5 and vehicle.velocity[1] < 0.5:
        
        # give the drone time to stop and assure its speed has further decreased
        time.sleep(2)
        if vehicle.velocity[0] < 0.3 and vehicle.velocity[1] < 0.3:
            stoppingLocation = vehicle.location.global_frame

            # for max. x seconds...
            timeout = time.time() + 10
            while(time.time() < timeout):
                # ... if currAvoided was incremented in the meantime (= object is near, avoidance)
                # and a movement of 1m or more is registered, and no waypoint has been reached,
                # then an avoidance maneuver is assumed 
                if (currAvoided > prevAvoided) and abs(get_distance_metres(stoppingLocation, vehicle.location.global_frame)) >= 1 and not drone.check_waypoint_reached(pos_tol=WAYPOINT_REACHED_TOL):  
                    return True
    return False

def detect_avoidance_maneuver():
    """
    Continuously investigates properties of the drone and returns whether an avoidance maneuver might have occurred.
    """
    global avoidance_maneuver_counter # this global variable is reset to zero by the main thread at every waypoint

    while True:
        if avoidance_detected(): 
            rospy.loginfo(f"\n\n<DRONE {DRONE_NO}>\n----- AVOIDANCE MANEUVER DETECTED! -----\n")
            avoidance_maneuver_counter += 1
            
            # for max. x seconds, wait for the avoidance to complete i.e. wait for the drone to speed up
            timeout = time.time() + 10
            while(time.time() < timeout):
                if vehicle.velocity[0] < 0.5 and vehicle.velocity[1] < 0.5:
                    rospy.loginfo(f"\n\n<DRONE {DRONE_NO}>\n----- Waiting for avoidance maneuver to complete... -----\n") 
                    time.sleep(2)
                else:
                    break # continue monitoring for next avoidance maneuver
        else:
            time.sleep(AVOIDANCE_MONITOR_FREQ) # reduce overhead




############################## INSTRUCTIONS ##############################

def main():
    """
    Main function with drone instructions.
    """
    global avoidance_maneuver_counter
    global travel_distance

    # initialize generic ROS node and create subscriber for lidar
    rospy.init_node("obs_avoider", anonymous=True)
    rospy.Subscriber(name="/spur/laser/scan",
                     data_class=LaserScan,
                     queue_size=1,
                     callback=laser_cb)

    # wait for FCU connection.
    drone.wait4connect()
    # wait for the mode to be switched manually: # drone.wait4start()  
    # set mode to guided
    drone.set_mode("GUIDED")
    # set speed (heavily affects recommended object detection distances)
    drone.set_speed(VELOCITY)
    # create local reference frame.
    drone.initialize_local_frame()
    # save initial launching location
    launchPoint = vehicle.location.global_frame
    # request takeoff
    drone.takeoff(TAKEOFF_HEIGHT)

############ WAYPOINT DEFINITIONS ############
    # yaw: 0 = forward, -90 = right, 0 = forward, 90 = left, 180 = backward, 0 = forward
    # mind that all coordinates apply to the local frame of the individual drone! 
    # i.e. [0, 0, 0, 0] is always the launchpoint of the drone, no matter where in the world that is

    #square: 
    #waypoints = [[0, 0, 3, 0], [5, 0, 3, -90], [5, 5, 3, 0], [0, 5, 3, 90], [0, 0, 3, 180], [0, 0, 3, 0]]

    #stationary:
    #waypoints = [[0, 0, 3, 3]]

    # A to B to A: 
    waypoints = [[0, 0, 3, 0], [0, 100, 3, 0], [0, 0, 3, 180], [0, 0, 3, 0]]

    # B to A to A: (opposite path)
    #waypoints = [[0, 100, 3, 180], [0, 0, 3, 180], [0, 100, 3, 0], [0, 100, 3, 180]]

    # two drones from separate launchpoint to same target
    # DRONE 1 #waypoints = [[0, 0, 3, 0], [15, 100, 3, 0], [0, 0, 3, 180], [0, 0, 3, 0]]
    # DRONE 2 #waypoints = [[0, 0, 3, 0], [-15, 100, 3, 0], [0, 0, 3, 180], [0, 0, 3, 0]]

################################################

    waypointsCount = len(waypoints)

    # specify control loop rate 
    rate = rospy.Rate(3)

    # start distance measuring in the background
    distance_measuring = threading.Thread(target=get_distance_travelled)
    distance_measuring.daemon = True
    rospy.loginfo("Starting distance measuring thread...")
    distance_measuring.start()

    # start avoidance behaviour monitoring in the background
    avoidance_monitoring = threading.Thread(target=detect_avoidance_maneuver)
    avoidance_monitoring.daemon = True
    rospy.loginfo("Starting avoidance monitoring thread...")
    avoidance_monitoring.start()
    
    if(waypointsCount > 0):

        # initialise variables and lists
        i = 0
        elapsedTimes = np.zeros(waypointsCount)
        waypointDistances = np.zeros(waypointsCount)
        travelDistances = np.zeros(waypointsCount)
        maneuverCounts = np.zeros(waypointsCount)
        startTime = time.time()
        
        while i < waypointsCount:
            # move to next waypoint
            drone.set_destination(x=waypoints[i][0], y=waypoints[i][1], z=waypoints[i][2], psi=waypoints[i][3])

            rate.sleep()
            if drone.check_waypoint_reached(pos_tol=WAYPOINT_REACHED_TOL):
                # print(f"waypoint reached: x={waypoints[i][0]}, y={waypoints[i][1]}, z={waypoints[i][2]}, psi={waypoints[i][3]}\n"+
                # f"at global pos: {vehicle.location.global_frame}\n"+
                # f"at local pos: {drone.get_current_location()}\n")

                ### waypoint i was reached
                # log time
                currTime = time.time()
                currElapsedTime = currTime - startTime
                elapsedTimes[i] = currElapsedTime
                
                # log distance to waypoint
                currPoint = vehicle.location.global_frame
                currWaypointDistance = get_distance_metres(launchPoint, currPoint)
                waypointDistances[i] = currWaypointDistance

                # log distance travelled to waypoint
                travelDistances[i] = travel_distance

                # log avoidance maneuver count at waypoint
                maneuverCounts[i] = avoidance_maneuver_counter

                # console output
                print_current_logs(elapsedTimes[i], waypointDistances[i], travelDistances[i], maneuverCounts[i], i, DRONE_NO)

                # update / reset variables
                i += 1
                launchPoint = currPoint
                startTime = currTime  # slightly inaccurate, but this is not critical
                avoidance_maneuver_counter = 0
                travel_distance = 0

        ### all waypoints have been reached
        # land
        drone.land()
        rospy.loginfo(f"\n<DRONE {DRONE_NO}>")
        rospy.loginfo(CGREEN2 + "All waypoints reached, landing now." + CEND)

        # MR validation
        validate_mrs(elapsedTimes, travelDistances, maneuverCounts, DELTA_T, DELTA_D, DRONE_NO)
        # debug
        rospy.loginfo(f"<DRONE {DRONE_NO}> TOTAL TIME TAKEN: {elapsedTimes[1] + elapsedTimes[2]}")
        rospy.loginfo(f"<DRONE {DRONE_NO}> TOTAL MANEUVER COUNT: {maneuverCounts[1] + maneuverCounts[2]}")


if __name__ == '__main__':
    try:
        main()
    except KeyboardInterrupt:
        exit()
