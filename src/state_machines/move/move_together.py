import rospy
import smach
import smach_ros
import smach_ros.monitor_state
from std_msgs.msg import String

from collections import deque

from dao.moveDao import selectMoveDataByScene # mySQL
from dao.RobotDao import updateRobotDisplacement, displacementByRobotID # mySQL

class MoveRequest(smach.State):
    def __init__(self, direction:String):
        smach.State.__init__(self, outcomes=["done", "none"],
                                    input_keys=['command', 'scene', 'robot_list'],
                                    output_keys=['robot_list'])
        
        self.direction = direction # goal(move by scene), home(move opposite direction from scene)

        self.move_pub = rospy.Publisher('/scene_manager/move_req', String, queue_size=1)
        self.go_home_pub = rospy.Publisher('/scene_manager/go_home', String, queue_size=1)
        self.request_robot_list = []

    def execute(self, user_data):
        move_flow = deque()

        full_cmd_list = str(user_data.command).split()

        self.request_robot_list = full_cmd_list[2:]
        
        if self.direction == "backward":
            move_flow = selectMoveDataByScene(user_data.scene, isOpposite=True, robot_list=self.request_robot_list)
            
            sorted_list = list(move_flow)
            sorted_list.sort(key=lambda x: x.split()[2])
            move_flow = deque(sorted_list)

        else :
            move_flow = selectMoveDataByScene(user_data.scene, isOpposite=False, robot_list=self.request_robot_list)

        user_data.robot_list = []

        if len(move_flow) == 0:
            return 'none'
        
        while move_flow:
            if self.direction == "backward":
                rospy.sleep(3)
            
            rospy.sleep(0.3)
            
            goal_data = move_flow.popleft() 
            
            self.move_pub.publish(goal_data)

            rospy.loginfo("[MoveTogether] move_req is published now.")
            rospy.loginfo("[MoveTogether] data published now: %s", goal_data)

            user_data.robot_list.append(goal_data.split()[0])
        
        rospy.loginfo("[MoveTogether] robot_list is updated now (%s)", str(user_data.robot_list))

        return 'done'

class OnTheMove(smach_ros.MonitorState):
    def __init__(self):
        smach_ros.MonitorState.__init__(self, '/scene_manager/move_res', String, self.check_leftover,
                                        input_keys=['command', 'scene', 'robot_list'],
                                        output_keys=['robot_list'])
        
    def check_leftover(self, user_data, res_msg):
        result = str(res_msg.data).split()

        if result[0] in user_data.robot_list:
            # update current displacement of robot
            disX, disZ = displacementByRobotID(robotID=result[0])
            newDisX = float(disX) + float(result[1]) * float(result[2])
            newDisZ = float(disZ) + float(result[1]) * float(result[7])

            updateRobotDisplacement(robotID=result[0], displacementX=newDisX, displacementZ=newDisZ)

            user_data.robot_list.remove(result[0])
            rospy.loginfo(f"robot %s arrived", result[0])
            rospy.loginfo("[MoveTogether] robot_list is updated now (%s)", str(user_data.robot_list))
        
        if len(user_data.robot_list) > 0:
            return True
        
        return False

class Arrive(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['done'],
                                    input_keys=['scene', 'robot_list'],
                                    output_keys=['robot_list'])
    def execute(self, user_data):
        #[todo] error handling (fail)
        return 'done'            

class MoveTogetherSM(smach.StateMachine):
    def __init__(self, direction:String):
        smach.StateMachine.__init__(self, outcomes=["arrive"],
                                    input_keys=['command', 'scene', 'robot_list'],
                                    output_keys=['robot_list'])
        
        with self:
            self.add('MOVE_REQUEST', MoveRequest(direction=direction),
                     transitions={'done': 'ON_THE_MOVE',
                                  'none': "arrive"})
            self.add('ON_THE_MOVE', OnTheMove(),
                     transitions={'invalid': 'ARRIVE',
                                'valid': 'ON_THE_MOVE',
                                'preempted':'ON_THE_MOVE'})
            self.add('ARRIVE', Arrive(),
                     transitions={
                         'done': 'arrive'
                     })
