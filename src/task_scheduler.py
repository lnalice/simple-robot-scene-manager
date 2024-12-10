import smach

from state_machines.tasks.task_consumer import TaskConsumerSM

class TaskSchedulerSM(smach.StateMachine):
    def __init__(self):
        smach.StateMachine.__init__(self, outcomes=['unknown', 'none', 'end'],
                                     input_keys=['sync_queue'],
                                     output_keys=['sync_queue'])
        
        self.concurrence = smach.Concurrence(
            outcomes=['exit', 'done'],
            default_outcome='exit',
            child_termination_cb=lambda outcome_map: False,
            outcome_cb=lambda outcome_map: 'exit',
            input_keys=[], output_keys=[]
        )

        self.set_initial_state(['CONSUME'])
        self.userdata.command = ''
        self.userdata.scene = ''
        self.userdata.robot_list = []

        with self.concurrence:
            smach.Concurrence.add('CONSUME', TaskConsumerSM())
    
    def execute(self, parent_ud=...):
        return super().execute(parent_ud)