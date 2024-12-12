import rospy
import smach

import queue

from config.actionList import baseAction

class TaskConsumer(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['done', 'none'],
                             io_keys=['sync_queue', 'robot_queue'])

    def enqueue(self, robot: str, cmd: str, queues: dict):
        if robot not in queues:
            queues[robot] = queue.Queue() # 새로운 큐 생성
            rospy.logwarn(f"Created new queue for a robot {robot}.")

        queues[robot].put(cmd) # insert an data (ex. MOVE tb3_0 seconds linX angZ delay)

    def execute(self, user_data):
        try:
            rospy.sleep(1)
            
            robot_list = set()
            task = user_data.sync_queue.get()
            
            for cmd in task:
                cmd_list = cmd.split()
                # action = cmd_list[0] # ex. MOVE
                robot = cmd_list[1] # ex. tb3_0
                # data = cmd_list[2:] # ex. seconds linX angZ delay

                robot_list.add(robot)
                
                self.enqueue(robot, cmd, user_data.robot_queues)

            rospy.loginfo(f"[TaskConsumer] These robots will be tracked for a task: %s.", robot_list)
            
            return 'done'

        except queue.Empty:
            rospy.loginfo(f"[TaskConsumer] Queue is empty, still waiting...")
            return 'none'
