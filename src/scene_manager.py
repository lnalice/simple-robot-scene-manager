#!/usr/bin/env python3

import rospy
import smach
import smach_ros

import argparse

from state_machines.move.move_together import MoveTogetherSM
from state_machines.spin.spin_together import SpinTogetherSM # It's for robot only to go forward
from state_machines.control.control_module import CtrlModuleSM
from state_machines.request_handler import RequestInterpreterSM
from state_machines.move.go_home import GoHomeSM
from state_machines.reset import ResetSM
                
class SceneManager:
    def __init__(self):

        rospy.init_node("scene_manager_node")

        # self.scene = "scene" + "_" + param.scene[0] #[todo] have to deal with scene list

        self.sm = smach.StateMachine(outcomes=["end"])
        self.sm.set_initial_state(['REQUEST'])
        self.sm.userdata.scene = "0"
        self.sm.userdata.robot_list = []
        self.sm.userdata.command = ""

        with self.sm:
            smach.StateMachine.add('REQUEST', RequestInterpreterSM(),
                                   transitions={'scene': 'SCENE_MOVE',
                                                'move': 'MOVE',
                                                'module': 'CTRL_MODULE',
                                                'home': 'RESET'})
            """
            [ follow the scene ]
            """
            smach.StateMachine.add('SCENE_MOVE', MoveTogetherSM(direction="forward"),
                                   transitions={'arrive': 'SCENE_CTRL_MODULE'})
            smach.StateMachine.add('SCENE_CTRL_MODULE', CtrlModuleSM(direction="forward"),
                                   transitions={'complete': 'SCENE_RESET_MODULE'})
            smach.StateMachine.add('SCENE_RESET_MODULE', CtrlModuleSM(direction="backward"),
                                   transitions={'complete': 'SCENE_RESET_MOVE'})
            smach.StateMachine.add('SCENE_RESET_MOVE', MoveTogetherSM(direction="backward"),
                                   transitions={'arrive': 'REQUEST'})
            # smach.StateMachine.add('SCENE_RESET', ResetSM(),
            #                        transitions={'complete': 'REQUEST'})
            """
            [ custom command ]
            """
            smach.StateMachine.add('MOVE', MoveTogetherSM(direction="forward"),
                                   transitions={'arrive': 'REQUEST'})
            smach.StateMachine.add('CTRL_MODULE', CtrlModuleSM(direction="forward"),
                                   transitions={'complete': 'REQUEST'})
            smach.StateMachine.add('RESET', CtrlModuleSM(direction="backward"),
                                   transitions={'complete': 'RESET_FIN'})
            smach.StateMachine.add('RESET_FIN', MoveTogetherSM(direction="backward"),
                                   transitions={'arrive': 'REQUEST'})
            # smach.StateMachine.add('RESET', ResetSM(),
            #                        transitions={'complete': 'REQUEST'})
            
            """
            ** If you want robot only to go forward

            smach.StateMachine.add('MOVE', MoveTogetherSM(direction="forward"),
                                   transitions={ 'arrive': 'CTRL_MODULE'})
            smach.StateMachine.add('CTRL_MODULE', CtrlModuleSM(),
                                   transitions={'complete': 'SPIN'})
            smach.StateMachine.add("SPIN", SpinTogetherSM(),
                                   transitions={'arrive': 'COME_BACK_FORWARD'})
            smach.StateMachine.add('COME_BACK_FORWARD', MoveTogetherSM(direction="forward"),
                                   transitions={'arrive': 'end'})
            """

# parser = argparse.ArgumentParser(description="robot specification",
#                                  formatter_class=argparse.ArgumentDefaultsHelpFormatter)
# parser.add_argument(
#     '-s', '--scene', required=True, nargs='+', help='Scene Number (1, 2, 3)') # ex) 1 3 or 1
# args = parser.parse_args()

if __name__ == "__main__":

    # simple_traveler = SceneManager(param=args)
    simple_traveler = SceneManager()

    sis = smach_ros.IntrospectionServer('scene_manager_node', simple_traveler.sm, '/scene_manager')
    sis. start()

    outcome = simple_traveler.sm.execute()
    rospy.loginfo("[Scene Manager] final state is %s. done.", outcome)
    
    rospy.spin()
    sis.stop()
    