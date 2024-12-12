import rospy
import smach
from std_msgs.msg import String

import queue

class TaskDirector(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['done', 'none'],
                             input_keys=['robot_queue', 'idle_robots'],
                             output_keys=['robot_queue'])
        
        # self.publishers = {}  # [todo] publisher 미리 생성해서 관리

    def dequeue(self, idle_robots: set, queues: dict):
        for key in list(queues.keys()):
            
            sub_queue = queues[key]

            if not sub_queue.empty() and key in idle_robots:
                cmd = sub_queue.get()
                robot_name = cmd.split()[1]

                publisher =  rospy.Publisher('/task_scheduler/' + robot_name, String, queue_size=1)
                rospy.sleep(0.1)
                publisher.publish(cmd)

            else:
                del sub_queue # 빈 큐 삭제

    def execute(self, user_data):
        try:
            rospy.sleep(1)
            
            self.dequeue(user_data.idle_robots, user_data.robot_queues)

            rospy.loginfo(f"[TaskConsumer] All queues iterated. Waiting 1 second.")
            
            return 'done'

        except queue.Empty:
            rospy.loginfo(f"[TaskConsumer] Queue is empty, still waiting...")
            
            return 'none'
