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
        {'x': 1, 'y': 0},
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

    posters = goal_queue.posters

    wanted_poster = max(posters,key=lambda p: p.prize)

    

    brain.cylinder_manager = CylinderManager(wanted_poster,cylinders_to_approach)





    rate = rospy.Rate(1)
        