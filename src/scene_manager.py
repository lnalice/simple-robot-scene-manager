import rospy
import smach
import smach_ros

import argparse

from state_machines.test.task_allocator import TaskAllocatorSM
from state_machines.test.task_planner import TaskPlannerSM

class ScenePlanner:
    def __init__(self, param):

        self.scene_id = param.scene
        smach.StateMachine.__init__('scene_manager_node' + '_' + self.scene_id)

        self.task_manager_sm = smach.StateMachine(outcomes=['end'])

        with self.task_manager_sm:
            smach.StateMachine.add('TASK_PLANNER', TaskPlannerSM(),
                                   transitions={'proceeding': 'TASK_ALLOCATOR',
                                                'end': 'end'})
            smach.StateMachine.add('TASK_ALLOCATER', TaskAllocatorSM(),
                                   transision={'done': 'TASK_PLANNER'})


parser = argparse.ArgumentParser(description="robot specification",
                                 formatter_class=argparse.ArgumentDefaultsHelpFormatter)
parser.add_argument(
    '-s', '--scene', required=True, help='Scene Number (1, 2, 3)') # ex) tb3_1
args = parser.parse_args()

if __name__ == "__main__":
    simple_traveler = ScenePlanner(param=args)
    simple_traveler.move_action()
    rospy.spin()