import smach

from tasks.task_consumer import TaskConsumer
from tasks.robot_monitor import RobotMonitor
from tasks.task_director import TaskDirector

class TaskSchedulerSM(smach.StateMachine):
    def __init__(self):
        smach.StateMachine.__init__(self, outcomes=['unknown', 'none', 'end'],
                                     input_keys=['sync_queue'],
                                     output_keys=['sync_queue'])
        
        self.concurrence = smach.Concurrence(
            outcomes=['exit', 'done'],
            default_outcome='done',
            outcome_cb=None,
        )

        self.set_initial_state(['CONSUME'])
        self.userdata.robot_queues = {} # 로봇 별 큐를 저장
        self.userdata.idle_robots = set() # 이용가능한 로봇 저장

        with self.concurrence:
            smach.Concurrence.add('CONSUME', TaskConsumer())
            smach.Concurrence.add('MONITOR', RobotMonitor())
            smach.Concurrence.add('DIRECTOR', TaskDirector())
    
    def execute(self, parent_ud=...):
        return super().execute(parent_ud)