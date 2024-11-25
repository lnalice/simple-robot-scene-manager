#!/usr/bin/env python3

import smach

import queue

from state_machines.move.move_together import MoveTogetherSM
from state_machines.control.control_module import CtrlModuleSM
from state_machines.tasks.task_consumer import TaskConsumerSM

class TaskSchedulerSM:
    def __init__(self, sync_queue: queue.Queue):

        self.sm = smach.StateMachine(outcomes=['end'])
        self.sm.set_initial_state(['REQUEST'])
        self.sm.userdata.command = ''
        self.sm.userdata.scene = ''
        self.sm.userdata.robot_list = []

        with self.sm:
            smach.StateMachine.add('REQUEST', TaskConsumerSM(sync_queue),
                                   transitions={'move': 'MOVE',
                                                'module': 'CTRL_MODULE',
                                                'home': 'HOME',
                                                'min': 'MINIMIZE_MODULE'})

            smach.StateMachine.add('MOVE', MoveTogetherSM(direction="forward"),
                                   transitions={'arrive': 'REQUEST'})
            smach.StateMachine.add('CTRL_MODULE', CtrlModuleSM(direction="forward"),
                                   transitions={'complete': 'REQUEST'})
            smach.StateMachine.add('HOME', MoveTogetherSM(direction="backward"),
                                   transitions={'arrive': 'REQUEST'})
            smach.StateMachine.add('MINIMIZE_MODULE', CtrlModuleSM(direction="backward"),
                                   transitions={'complete': 'REQUEST'})