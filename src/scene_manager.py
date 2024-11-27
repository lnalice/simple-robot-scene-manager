#!/usr/bin/env python3

import signal

import rospy
import smach

import queue

from task_assignment import TaskAssignmentSM
from task_scheduler import TaskSchedulerSM

def signal_handler(signum, frame):
    res = input("\n[Scene Manager] Ctrl-c was pressed. Do you want to exit? (y/n) ")
    if res =='y':
        exit(1)

signal.signal(signal.SIGINT, signal_handler)

class SceneManager:
    def __init__(self):

        rospy.init_node("scene_manager_node")

        self.concurrence = smach.Concurrence(
            outcomes=['exit', 'done'],
            default_outcome='exit',
            child_termination_cb=lambda outcome_map: False,
            outcome_cb=lambda outcome_map: 'exit',
            input_keys=[], output_keys=[]
        )

        self.concurrence.userdata.sync_queue = queue.Queue()

        with self.concurrence:
            smach.Concurrence.add('TASK_ASSIGNMENT', TaskAssignmentSM())
            smach.Concurrence.add('TASK_SCHEDULER', TaskSchedulerSM())

if __name__ == "__main__":
    scene_manager = SceneManager()

    outcome = scene_manager.concurrence.execute()
    rospy.loginfo("[Scene Manager] final state is %s. done.", outcome)
    
    rospy.spin()