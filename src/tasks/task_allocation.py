import rospy
import smach
import smach.state

import queue

from tasks.logic.random_allocation import randomAllocation
from helper.context2task import context2task

class GetContext(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['done', 'none'],
                             io_keys=['context', 'roleData', 'context_queue'])
        
    def execute(self, user_data):
        try:
            user_data.context, user_data.roleData = user_data.context_queue.get()

            return 'done'

        except user_data.context_queue.Empty:
            rospy.loginfo(f"[TaskAllocation] Queue is empty, still waiting...")
            return 'none'

class SelectRobots(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=["done", "rejected"],
                             io_keys=['context', 'roleData', 'task'])

    def execute(self, user_data):
        robots = randomAllocation(user_data.roleData) # temporal algorithm (random)

        if not robots:
            return 'rejected'
        
        user_data.task = context2task(contextID=user_data.context, robotsByRole=robots)

        return 'done'
    
class WriteTask(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['done'],
                             io_keys=['task', 'context', 'roleData', 'sync_queue'])

    def execute(self, user_data):
        if not user_data.task:
            return 'done'
        
        sync_queue: queue.Queue = user_data.sync_queue
        sync_queue.put(user_data.task)
        rospy.loginfo(f"[TaskProducer] I put new task into the queue. ({user_data.task})")

        user_data.task = [] # reset task
        user_data.context = ""
        user_data.roleData = {}

        return 'done'

class TaskAllocationSM(smach.StateMachine):
    def __init__(self):
        smach.StateMachine.__init__(self, outcomes=['done', 'none'],
                                    input_keys=['context_queue', 'sync_queue'],
                                    output_keys=['context_queue', 'sync_queue'])

        self.userdata.context = "" # ex) FRIEND
        self.userdata.roleData = {} # ex) { 'table': 10, 'chair': 15 }
        self.userdata.task = [] # ex) ["MODULE tb3_0 seconds linX angZ delay", "MOVE tb3_1 seconds linX angZ delay"]

        with self:
            smach.StateMachine.add('READ_CONTEXT', GetContext(),
                                   transitions={'none': 'READ_CONTEXT',
                                                'done': 'ROBOT_ALLOCATION'})
            smach.StateMachine.add('ROBOT_ALLOCATION', SelectRobots(),
                                   transitions={'done': 'WRITE_TASK',
                                                'rejected': 'READ_CONTEXT'})
            smach.StateMachine.add('WRITE_TASK', WriteTask(),
                                   transitions={'done': 'READ_CONTEXT'})