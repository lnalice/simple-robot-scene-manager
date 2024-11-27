import smach
import smach.state

import queue

from state_machines.tasks.task_producer import TaskProducerSM


class TaskAssignmentSM(smach.StateMachine):
    def __init__(self):
        smach.StateMachine.__init__(self, outcomes=['exit', 'done'],
                                     input_keys=['sync_queue'],
                                     output_keys=['sync_queue'])
        self.set_initial_state(['PRODUCE'])

        with self:
            """
            [to-do] 'role assignment' -> 'task allocation' draft
            """
            # self.add('ROLE_SETTING', RoleSettingSM(),
            #          transitions={'invalid': 'TASK_ALLOCATION',
            #                       'valid': 'ROLE_SETTING',
            #                       'preempted':'ROLE_SETTING'})
            # self.add('TASK_ALLOCATION', TaskAllocationSM(),
            #          transitions={
            #              'busy': 'TASK_ALLOCATION',
            #              'done': 'PRODUCE'})

            smach.StateMachine.add('PRODUCE', TaskProducerSM())

    def execute(self, parent_ud=...):
        return super().execute(parent_ud)