#!/usr/bin/python3

from goals_task3_improved import GoalQueue, explore_goals
import rospy
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from cylinder_manager import CylinderManager



class Brain:
    def __init__(self):
        self.RUNNING = True

        self.map_goals =  [
        #{'x': 0, 'y': -1},
        {'x': 2.5, 'y': -0.5},
        {'x': 1, 'y': 0},
        # {'x': 2.5, 'y': 0},
        #{'x': 2.5, 'y': 1.3},
        {'x': 1, 'y': 2.5},
        #{'x': 0.12, 'y': -1.6},
        #{'x': 0.1, 'y': -1.5},
        #{'x': 1.0, 'y': -1.7},
        #{'x': 3.1, 'y': -1.05},
        #{'x': 2.35, 'y': 1.85},
        #{'x': -1.0, 'y': 1.1}

        
        # add more goals as needed
    ]
    # add more goals as needed
        self.gq = GoalQueue(self.map_goals,num_faces=3)
        self.cylinder_manager = None



if __name__ == '__main__':
    # try:
    rospy.init_node('task1_goals', anonymous=True)

    brain = Brain()

    goal_queue = brain.gq

    goal_queue.print_goals()

    cylinders_to_approach = explore_goals(goal_queue)

    print("cylinders to approach:")
    for i, cyl in enumerate(cylinders_to_approach):
        print(f"Cylinder {i}: color: {cyl.color}")
        print(f"Cylinder {i}: pose: {cyl.pose}")

    posters = goal_queue.posters

    for p in posters:
        print("Poster image:",p.image)
        print("Poster prize:",p.prize)
        print("Poster color:",p.color)

    wanted_poster = max(posters,key=lambda p: p.prize)

    prison_color = wanted_poster.color
    print("Wanted prison color:", prison_color)
    

    brain.cylinder_manager = CylinderManager(wanted_poster,cylinders_to_approach)
    brain.cylinder_manager.find_prisoner()

    # Ring Color --> Ring Pose
    # TODO
    rings = goal_queue.rings()

    print("Rings:")
    for i, ring in enumerate(rings):
        print(f"Rings {i}: color: {ring.color}")
        print(f"Rings {i}: pose: {ring.pose}")

    prison_ring  = [ring for ring in rings if ring.color == prison_color][0]


    print(f"Prison ring {prison_ring}")


    # ParkingManager()

        # 1. Approach {prison_color} ring

        # 2. Park the robot
            # ParkingDetector()









    rate = rospy.Rate(1)
        