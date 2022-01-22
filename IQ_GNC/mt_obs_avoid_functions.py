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

import rospy
from sensor_msgs.msg import LaserScan
from iq_gnc.py_gnc_functions import *
from geometry_msgs.msg import Point
import math





############################################################ FUNCTIONS ############################################################


def validate_mr_1(elapsedTimes, travelDistances, maneuverCounts, delta_t, delta_d):
    """
    MR 1: No obstacles, one drone, elapsed time and travelled distance from A to B should be roughly equal as from B to A
    and no avoidance maneuvers are to be expected.
    """   
    timeDiff = float(elapsedTimes[1] - elapsedTimes[2])
    distDiff = float(travelDistances[1] - travelDistances[2])
    return (abs(timeDiff) <= delta_t) and (abs(distDiff) <= delta_d) and maneuverCounts[1] == 0 and maneuverCounts[2] == 0

def validate_mr_2(maneuverCounts):
    """
    MR 2: Obstacles between the points, one drone, expect at least one maneuver for each direction.
    """
    return (maneuverCounts[1] >= 1) and (maneuverCounts[2] >= 1)

def validate_mr_3(maneuverCounts):
    """
    MR 3: No obstacles, two drones, expect at least one maneuver per drone if they have the same target.
    (Each drone runs an instance of the same script. So the same validation method might fail for one drone and pass for the other)
    """
    return maneuverCounts[1] >= 1 or maneuverCounts[2] >= 1

#def validate_mr_4() uses same logic as validate_mr_1, on a per-drone basis

def validate_mrs(elapsedTimes, travelDistances, maneuverCounts, delta_t, delta_d, drone_no):
    """
    Calls all mr validation methods and prints their output to stdout.
    """
    print(f"\n---------- <DRONE {drone_no}> MR Validation ----------\n"+
    f"MR 1: {validate_mr_1(elapsedTimes, travelDistances, maneuverCounts, delta_t, delta_d)}\n"+
    f"MR 2: {validate_mr_2(maneuverCounts)}\n"+
    f"MR 3: {validate_mr_3(maneuverCounts)}\n"+
    f"MR 4: {validate_mr_1(elapsedTimes, travelDistances, maneuverCounts, delta_t, delta_d)}\n")

def print_current_logs(currElapsedTime, currWaypointDistance, currTravelDistance, currManeuversCount, i, drone_no):
    """
    Function for console output.
    """
    rospy.loginfo("\n==========================================================================================================\n"+
    f"<DRONE {drone_no}>\n"+
    f"Waypoint {i} has been reached!\n"+
    f"Shortest path to Waypoint:\t\t{currWaypointDistance:.3f}m\n"+
    f"Distance travelled:\t\t\t{currTravelDistance:.3f}m\n"+
    f"Elapsed wall-clock time:\t\t{currElapsedTime:.3f}s\n"+
    f"Number of avoidance maneuvers:\t\t{currManeuversCount:.0f}\n"+
    "==========================================================================================================\n")

def get_distance_metres(aLocation1, aLocation2):
    """
    Returns the ground distance in metres between two LocationGlobal objects.
    This method is an approximation, and will not be accurate over large distances and close to the 
    earth's poles. It comes from the ArduPilot test code: 
    https://github.com/diydrones/ardupilot/blob/master/Tools/autotest/common.py
    """
    dlat = aLocation2.lat - aLocation1.lat
    dlong = aLocation2.lon - aLocation1.lon
    return math.sqrt((dlat*dlat) + (dlong*dlong)) * 1.113195e5

# def waypoint_reached(waypoint, tol):
#     """
#     Checks whether the given waypoint has been reached.
#     (The built-in function seems to misbehave)
#     """
#     return get_distance_metres(drone.get_current_location(), waypoint) <= tol
