import smach
import smach.state

import queue

from tasks.task_allocation import TaskAllocationSM
from state_machines.context_monitor import ContextMonitorSM


class TaskAssignmentSM(smach.StateMachine):
    def __init__(self):
        smach.StateMachine.__init__(self, outcomes=['exit', 'fail'],
                                     input_keys=['sync_queue'],
                                     output_keys=['sync_queue'])
        # self.set_initial_state(['PRODUCE'])

        self.userdata.context_queue = queue.Queue()

        concurrence = smach.Concurrence(
            outcomes=['exit', 'fail'],
            default_outcome='exit',
            input_keys=['context_queue', 'sync_queue'],
            output_keys=['context_queue', 'sync_queue']
        )

        with self:
            self.add('TASK_ASSIGNMENT_CC', concurrence)
            with concurrence:
                smach.Concurrence.add('CONTEXT_MONITOR', ContextMonitorSM())
                smach.Concurrence.add('TASK_ALLOCATION', TaskAllocationSM())

    def execute(self, parent_ud=...):
        return super().execute(parent_ud)