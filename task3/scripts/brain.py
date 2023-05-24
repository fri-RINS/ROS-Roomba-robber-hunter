#!/usr/bin/python3

from goals_task3_improved import GoalQueue, explore_goals
import rospy
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from cylinder_manager import CylinderManager
from move_arm import Arm_Mover
from parking_manager import ParkingManager


class Brain:
    def __init__(self):
        self.RUNNING = True

        self.map_goals =  [
        {'x': -1.0, 'y': 0},
        {'x': 0, 'y': -1},
        {'x': 1, 'y': -1.2},
        {'x': 2.5, 'y': -0.5},
        {'x': 1, 'y': 0.3},
        {'x': 1, 'y': 2.5},
        {'x': 2.5, 'y': 1.3},
        {'x': 0.1, 'y': -1.5},
        {'x': 1.0, 'y': -1.7},
        {'x': 0, 'y': 0},
    ]
        
        self.whole_map_goals =  [
        {'x': -1.0, 'y': 0},
        {'x': 0, 'y': -1},
        {'x': 0.15, 'y': -1.2},
        {'x': 1, 'y': -1.2},
        {'x': 3.5, 'y': -0.6},
        {'x': 2.3, 'y': 0.5},
        {'x': 2.15, 'y': 0.95},
        {'x': 1, 'y': 0.2},
        {'x': 2, 'y': 2.4},
        {'x': 1, 'y': 2.6},
        {'x': -1.4, 'y': 1.7},
        {'x': -0.65, 'y': 0.15},
        {'x': 0, 'y': 0},
    ]
        self.dummy =  [
        # {'x': -1.0, 'y': 0},
        # {'x': 0, 'y': -1},
        # {'x': 0.15, 'y': -1.2},
        # {'x': 1, 'y': -1.2},
        {'x': 3, 'y': -0.6},
        # {'x': 2.15, 'y': 0.95},
        # {'x': 1, 'y': 0.2},
        # {'x': 2, 'y': 2.4},
        # {'x': 0.35, 'y': 2.4},
        # {'x': -1.4, 'y': 1.7},
        # {'x': -0.65, 'y': 0.15},
        # {'x': 0, 'y': 0},
    ]
    # add more goals as needed
        self.gq = GoalQueue(self.whole_map_goals,num_faces=2,num_posters=2,num_rings=4,num_cylinders=4)
        self.cylinder_manager = None
        self.am = Arm_Mover()
        self.parking_manager = None


if __name__ == '__main__':
    # try:
    rospy.init_node('brain_node', anonymous=True)


    brain = Brain()
    #retract arm at the start
    brain.am.retract_camera()

    goal_queue = brain.gq

    goal_queue.print_goals()

    #returns cylinders that need to be approached, after everything was detected.
    cylinders_to_approach = explore_goals(goal_queue)

    print("cylinders to approach:")
    for i, cyl in enumerate(cylinders_to_approach):
        print(f"Cylinder {i}: color: {cyl.color}")
        print(f"Cylinder {i}: pose: {cyl.pose}")

    rings = goal_queue.rings
    print("Rings:")
    for i, ring in enumerate(rings):
        print(f"Rings {i}: color: {ring.color}")
        print(f"Rings {i}: pose: {ring.pose}")


    posters = goal_queue.posters
    print("Posters:")
    for p in posters:
        print("Poster prize:",p.prize)
        print("Poster color:",p.color)


    #poster with biggest prize, this one has to be taken to prison
    wanted_poster = max(posters,key=lambda p: p.prize)


    print("STARTING CYLINDER MANAGER..")

    brain.cylinder_manager = CylinderManager(wanted_poster,cylinders_to_approach)
    #cylinder approach works
    #TODO
    #image comparison!!
    brain.cylinder_manager.find_prisoner()

    # Extract prison ring from poster and start parking manager
    prison_color = wanted_poster.color
    print("Wanted prison color:", prison_color) 

    prison_ring  = [ring for ring in rings if ring.color == prison_color][0]
    print("Prison ring:",prison_ring)

    print("STARTING PARKING MANAGER..")
    #Parking manager ring approach works
    #TODO
    #Park the robot in the circle, using ParkingDetector()!!
    brain.parking_manager = ParkingManager(prison_ring.pose)
    brain.parking_manager.park()

    print("ROBOT HAS PARKED, GOODBYE!")

    rate = rospy.Rate(1)
        