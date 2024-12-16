import smach

from tasks.task_consumer import TaskConsumerSM
from tasks.robot_monitor import RobotMonitorSM
from tasks.task_director import TaskDirectorSM

class TaskSchedulerSM(smach.StateMachine):
    def __init__(self):
        smach.StateMachine.__init__(self, outcomes=['exit','fail'],
                                     input_keys=['sync_queue'],
                                     output_keys=['sync_queue'])

        # self.set_initial_state(['CONSUME'])
        self.userdata.robot_queues = {} # 로봇 별 큐를 저장
        self.userdata.idle_robots = set(['tb3_0']) # 이용가능한 로봇 저장

        concurrence = smach.Concurrence(
            outcomes=['exit', 'fail'],
            default_outcome='exit',
            input_keys=['sync_queue', 'robot_queues', 'idle_robots'],
            output_keys=['sync_queue', 'robot_queues', 'idle_robots']
        )
        with self:
            self.add('TASK_SCHEDULER_CC', concurrence)

            with concurrence:
                smach.Concurrence.add('TASK_CONSUMER', TaskConsumerSM())
                smach.Concurrence.add('ROBOT_MONITOR', RobotMonitorSM())
                smach.Concurrence.add('TASK_DIRECTOR', TaskDirectorSM())
