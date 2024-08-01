#!/usr/bin/env python3

import rospy
import smach
import smach_ros

import argparse

from state_machines.move.move_together import MoveTogetherSM
from state_machines.move.spin_together import SpinTogetherSM
                
class SceneManager:
    def __init__(self, param):

        rospy.init_node("scene_manager_node")

        # robot_id goal_pose.position(x,y) goal_pose.orientation(z,w)
        self.scene = "scene" + "_" + param.scene[0] #[todo] have to deal with scene list

        self.sm = smach.StateMachine(outcomes=["end"])
        self.sm.set_initial_state(['MOVE'])
        self.sm.userdata.scene = self.scene
        self.sm.userdata.robot_list = []

        with self.sm:
            smach.StateMachine.add('MOVE', MoveTogetherSM(),
                                   transitions={ 
                                       'arrive': 'SPIN'})
                                    #    'arrive': 'CTRL_MODULE'})
            # smach.StateMachine.add('CTRL_MODULE', CtrlModuleSM(),
            #                        transision={'done': 'COME_BACK'})
            smach.StateMachine.add("SPIN", SpinTogetherSM(),
                                   transitions={'arrive': 'COME_BACK'})
            smach.StateMachine.add('COME_BACK', MoveTogetherSM(),
                                   transitions={ 
                                       'arrive': 'end'})


parser = argparse.ArgumentParser(description="robot specification",
                                 formatter_class=argparse.ArgumentDefaultsHelpFormatter)
parser.add_argument(
    '-s', '--scene', required=True, nargs='+', help='Scene Number (1, 2, 3)') # ex) 1 3 or 1
args = parser.parse_args()

if __name__ == "__main__":
    simple_traveler = SceneManager(param=args)

    sis = smach_ros.IntrospectionServer('scene_manager_node', simple_traveler.sm, '/scene_manager')
    sis. start()

    outcome = simple_traveler.sm.execute()
    # simple_traveler.move_action()
    rospy.loginfo("[Scene Manager] final state is %s. done.", outcome)
    
    rospy.spin()
    sis.stop()
    