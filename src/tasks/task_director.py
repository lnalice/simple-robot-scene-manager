import rospy
import smach
from std_msgs.msg import String

import queue

class Task2Robot(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=["fail"],
                             input_keys=['robot_queues', 'idle_robots'],
                             output_keys=['robot_queues'])
        
    def dequeue(self, key:str, idle_robots: set, queues: dict):
        sub_queue = queues[key]
        
        if key in idle_robots:
            cmd = sub_queue.get()
            
            robot_name = cmd.split()[1]
            
            publisher =  rospy.Publisher('/task_scheduler/' + robot_name, String, queue_size=1)
            rospy.sleep(0.1)
            publisher.publish(cmd)
            rospy.loginfo("published")

        else:
            del sub_queue # 빈 큐 삭제
            rospy.loginfo("delete")
    
    def execute(self, user_data):
        try:
            while True:
                rospy.sleep(1)
                for key in list(user_data.robot_queues.keys()):
                    self.dequeue(key, user_data.idle_robots, user_data.robot_queues)
            
        except:
            return 'fail'
            
    

class TaskDirectorSM(smach.StateMachine):
    def __init__(self):
        smach.StateMachine.__init__(self, outcomes=['fail'],
                             input_keys=['robot_queues', 'idle_robots'],
                             output_keys=['robot_queues'])
        
        # self.publishers = {}  # [todo] publisher 미리 생성해서 관리

        with self:
            smach.StateMachine.add('TO_ROBOT', Task2Robot(),
                                   transitions={'fail':'fail'})