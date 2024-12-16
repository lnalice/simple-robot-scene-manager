import rospy
import smach
from std_msgs.msg import String

class Task2Robot(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=["fail"],
                             input_keys=['robot_queues', 'idle_robots'],
                             output_keys=['robot_queues'])
        
    def dequeue(self, key:str, idle_robots: set, queues: dict):
        sub_queue = queues[key]
        robot_name = ""
        
        if key in idle_robots:
            cmd = sub_queue.get()
            
            robot_name = cmd.split()[1]
            
            publisher =  rospy.Publisher('/task_scheduler/' + robot_name, String, queue_size=1)
            rospy.sleep(0.1)
            publisher.publish(cmd)
            rospy.logwarn(f"[TaskDirector] I published to {robot_name}. ({cmd})")

            if sub_queue.empty(): # 빈 큐 삭제
                del queues[key]
                rospy.logwarn(f"[TaskDirector] Deleted a queue for a robot {robot_name}.")
                rospy.loginfo(f"[TaskDirector] Robot queues remaining: {queues.keys()}")
    
    def execute(self, user_data):
        try:
            while True:
                rospy.sleep(3)
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