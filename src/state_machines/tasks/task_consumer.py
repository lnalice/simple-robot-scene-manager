import rospy
import smach

import queue

from state_machines.tasks.config.actionList import baseAction

class Task2State(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=list(baseAction) + ['unknown', 'none'],
                             io_keys=['sync_queue', 'scene', 'robot_list', 'command'])
        
    def execute(self, user_data):
        try:
            user_data.command = user_data.sync_queue.get()
            task_list = user_data.command.split()

            action = task_list[0] # MOVE or HOME or MODULE or SCENE
            user_data.scene = task_list[1] # ex) friend, module_50
        
            user_data.robot_list = task_list[2:]

            rospy.loginfo(f"[TaskConsumer] SceneManager will track these robots: %s.", (user_data.robot_list))
            rospy.loginfo(f"[TaskConsumer] I saved state \'%s\' in SceneManager", action)
            
            return action if action in baseAction else 'unknown'

        except queue.Empty:
            rospy.loginfo(f"[TaskConsumer] Queue is empty, still waiting...")
            return 'none'
        

class TaskConsumerSM(smach.StateMachine):
    def __init__(self):
        smach.StateMachine.__init__(self, 
                                    outcomes=list(baseAction) + ['unknown', 'none'],
                                    input_keys=['sync_queue', 'scene', 'robot_list', 'command'],
                                    output_keys=['sync_queue', 'scene', 'robot_list', 'command'])
        with self:
            smach.StateMachine.add('READ_TASK', Task2State(),
                                   transitions={'none': 'READ_TASK',
                                                'unknown': 'READ_TASK'})