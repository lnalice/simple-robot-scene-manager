#!/usr/bin/env python3

import rospy
import smach
import smach_ros

import argparse

from state_machines.move.move_together import MoveTogetherSM
                
class ScenePlanner:
    def __init__(self, param):

        rospy.init_node("scene_planner_node")

        # robot_id goal_pose.position(x,y) goal_pose.orientation(z,w)
        self.scene = "scene" + "_" + param.scene[0] #[todo] have to deal with scene list

        self.sm = smach.StateMachine(outcomes=["end"])
        self.sm.userdata.scene = self.scene
        self.sm.userdata.robot_list = []

        with self.sm:
            smach.StateMachine.add('MOVE', MoveTogetherSM(),
                                   transitions={ 
                                       'arrive': 'end'})
                                    #    'end': 'CTRL_MODULE'})
            # smach.StateMachine.add('CTRL_MODULE', CtrlModuleSM(self.ctrl_task_id),
            #                        transision={'done': 'CTRL_MODULE',
            #                                    'end':'COME_BACK'})
            # smach.StateMachine.add("COME_BACK",)


parser = argparse.ArgumentParser(description="robot specification",
                                 formatter_class=argparse.ArgumentDefaultsHelpFormatter)
parser.add_argument(
    '-s', '--scene', required=True, nargs='+', help='Scene Number (1, 2, 3)') # ex) 1 3 or 1
args = parser.parse_args()

if __name__ == "__main__":
    simple_traveler = ScenePlanner(param=args)
    outcome = simple_traveler.sm.execute()
    # simple_traveler.move_action()
    rospy.spin()
    