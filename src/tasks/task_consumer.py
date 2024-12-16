import rospy
import smach

import queue

class ReadTask(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=["done", "none", "fail"],
                             io_keys=["sync_queue", "robot_queues", "command"])
    
    def enqueue(self, robot: str, cmd: str, queues: dict):
        if robot not in queues:
            queues[robot] = queue.Queue() # 새로운 큐 생성
            rospy.logwarn(f"Created new queue for a robot {robot}.")

        queues[robot].put(cmd) # insert an data (ex. MOVE tb3_0 seconds linX angZ delay)
    
    def execute(self, user_data):
        try:
            rospy.sleep(1)
            
            robot_list = set()
            task = user_data.sync_queue.get(timeout=10)
            user_data.command = task
            
            rospy.loginfo(f"[TaskConsumer] These robots will be tracked for a task: %s.", user_data.command)
            self.enqueue("tb3_0", user_data.command, user_data.robot_queues)
            
            return 'done'

        except queue.Empty:
            rospy.loginfo(f"[TaskConsumer] Queue is empty, still waiting...")
            return 'none'


class TaskConsumerSM(smach.StateMachine):
    def __init__(self):
        smach.StateMachine.__init__(self, outcomes=['fail'],
                             input_keys=['sync_queue', 'robot_queues'],
                             output_keys=['sync_queue', 'robot_queues'])
        
        self.userdata.command = ""

        with self:
            smach.StateMachine.add('READ_TASK', ReadTask(),
                                   transitions={'done':'READ_TASK',
                                                'none': 'READ_TASK',
                                                'fail': 'fail'})