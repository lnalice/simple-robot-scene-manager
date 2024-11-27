import smach

from state_machines.move.move_together import MoveTogetherSM
from state_machines.control.control_module import CtrlModuleSM
from state_machines.tasks.task_consumer import TaskConsumerSM

class TaskSchedulerSM(smach.StateMachine):
    def __init__(self):
        smach.StateMachine.__init__(self, outcomes=['unknown', 'none', 'end'],
                                     input_keys=['sync_queue'],
                                     output_keys=['sync_queue'])
        
        self.set_initial_state(['CONSUME'])
        self.userdata.command = ''
        self.userdata.scene = ''
        self.userdata.robot_list = []

        with self:
            smach.StateMachine.add('CONSUME', TaskConsumerSM(),
                                   transitions={'MOVE': 'MOVE',
                                                'MODULE': 'CTRL_MODULE',
                                                'HOME': 'HOME',
                                                'MIN': 'MINIMIZE_MODULE'})

            smach.StateMachine.add('MOVE', MoveTogetherSM(direction="forward"),
                                   transitions={'arrive': 'CONSUME'})
            smach.StateMachine.add('CTRL_MODULE', CtrlModuleSM(direction="forward"),
                                   transitions={'complete': 'CONSUME'})
            smach.StateMachine.add('HOME', MoveTogetherSM(direction="backward"),
                                   transitions={'arrive': 'CONSUME'})
            smach.StateMachine.add('MINIMIZE_MODULE', CtrlModuleSM(direction="backward"),
                                   transitions={'complete': 'CONSUME'})
    
    def execute(self, parent_ud=...):
        return super().execute(parent_ud)