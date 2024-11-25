#!/usr/bin/env python3

import signal

import rospy
import smach
import smach_ros

from state_machines.move.move_together import MoveTogetherSM
from state_machines.control.control_module import CtrlModuleSM
from state_machines.request_handler import RequestInterpreterSM

def signal_handler(signum, frame):
    res = input("\n[Scene Manager] Ctrl-c was pressed. Do you want to exit? (y/n) ")
    if res =='y':
        exit(1)

signal.signal(signal.SIGINT, signal_handler)

class SceneManager:
    def __init__(self):

        rospy.init_node("scene_manager_node")

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
                                                'home': 'HOME',
                                                'min': 'MINIMIZE_MODULE',
                                                'reset': 'RESET'})
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

            """
            [ custom command ]
            """
            smach.StateMachine.add('MOVE', MoveTogetherSM(direction="forward"),
                                   transitions={'arrive': 'REQUEST'})
            smach.StateMachine.add('CTRL_MODULE', CtrlModuleSM(direction="forward"),
                                   transitions={'complete': 'REQUEST'})
            
            smach.StateMachine.add('HOME', MoveTogetherSM(direction="backward"),
                                   transitions={'arrive': 'REQUEST'})
            smach.StateMachine.add('MINIMIZE_MODULE', CtrlModuleSM(direction="backward"),
                                   transitions={'complete': 'REQUEST'})

            smach.StateMachine.add('RESET', CtrlModuleSM(direction="backward"),
                                   transitions={'complete': 'RESET_FIN'})
            smach.StateMachine.add('RESET_FIN', MoveTogetherSM(direction="backward"),
                                   transitions={'arrive': 'REQUEST'})

if __name__ == "__main__":

    simple_traveler = SceneManager()

    sis = smach_ros.IntrospectionServer('scene_manager_node', simple_traveler.sm, '/scene_manager')
    sis. start()

    outcome = simple_traveler.sm.execute()
    rospy.loginfo("[Scene Manager] final state is %s. done.", outcome)
    
    rospy.spin()
    sis.stop()