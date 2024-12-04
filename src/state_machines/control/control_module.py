import rospy
import smach
import smach_ros
import smach_ros.monitor_state
from std_msgs.msg import String

from collections import deque

from dao.sceneModuleDao import selectModuleDataByScene # mySQL
from dao.RobotDao import updateRobotModuleState, selectModuleStateByRobotID, updateRobotStatus # mySQL
from dao.moduleDao import selectDegreeByState, selectStateByDegree # mySQL

DISPLAY_TIME = 10.0
IDLE = "IDLE"

class ControlRequest(smach.State):
    def __init__(self, direction:String):
        smach.StateMachine.__init__(self, outcomes=["done", "none"],
                                    input_keys=['command', 'scene', 'robot_list'],
                                    output_keys=['robot_list'])
        self.ctrl_pub = rospy.Publisher('/scene_manager/ctrl_module_req', String, queue_size=1)

        self.direction = direction        
        self.request_robot_list = []

    def execute(self, user_data):
        ctrl_flow = deque()
        
        full_cmd_list = str(user_data.command).split()

        display_time = DISPLAY_TIME if full_cmd_list[0] == "SCENE" else 0

        self.request_robot_list = full_cmd_list[2:]

        if self.direction == "backward":
            ctrl_flow = selectModuleDataByScene(user_data.scene, isOpposite=True, robot_list=self.request_robot_list)
            rospy.loginfo("[CtrlModule] The waiting time has been set. I'll wait \"%s\" seconds...", display_time)
            rospy.sleep(display_time)
        
        else:
            ctrl_flow = selectModuleDataByScene(user_data.scene, isOpposite=False, robot_list=self.request_robot_list)

        user_data.robot_list =[]

        if len(ctrl_flow) == 0:
            return 'none'
        
        while ctrl_flow:
            rospy.sleep(0.3)

            goal_data = ctrl_flow.popleft()

            self.ctrl_pub.publish(goal_data)

            rospy.loginfo("[CtrlModule] ctrl_module_req is published now: %s", goal_data)

            robotID = goal_data.split()[0]

            # 로봇 상태 업데이트 (모듈제어 중)
            updateRobotStatus(robotID=robotID, status=full_cmd_list[0])
            rospy.logwarn(f"[CtrlModule] robot {robotID}'s status updated to {full_cmd_list[0]}.")

            user_data.robot_list.append(goal_data.split()[0])
        
        rospy.loginfo("[CtrlModule] robot_list is updated now (%s)", str(user_data.robot_list))

        return 'done'
        
class InControl(smach_ros.MonitorState):
    def __init__(self):
        smach_ros.MonitorState.__init__(self, '/scene_manager/ctrl_module_res', String, self.check_leftover,
                                        input_keys=['scene', 'robot_list'],
                                        output_keys=['robot_list'])
        
    def check_leftover(self, user_data, res_msg):
        result = str(res_msg.data).split()

        rospy.loginfo("[CtrlModule] result %s", str(result))

        if result[0] in user_data.robot_list:
            newState:float = 0

            # update current module state of robot
            try:
                (moduleState,) = selectModuleStateByRobotID(robotID=result[0]) # 현재 state 불러오기

                cur_degZ, cur_degX = selectDegreeByState(state= moduleState) # 현재 state기준 degree 불러오기

                # 목표 degree = 각도 변위값 + 현재 degree
                goal_degZ = float(result[1]) + cur_degZ
                goal_degX = float(result[2]) + cur_degX
                
                # 업데이트할 state = 목표 degre로 state불러오기
                (newState,) = selectStateByDegree(degZ=goal_degZ, degX=goal_degX)
                
                # 새로운 state로 업데이트
                updateRobotModuleState(robotID=result[0], moduleState=float(newState))
                rospy.logwarn(f"[CtrlModule] Robot {result[0]}'s module state has now been updated to {newState}.")
            except:
                rospy.logerr("[CtrlModule] Failed to update the robot's module state.")

            user_data.robot_list.remove(result[0])
            rospy.loginfo(f"[CtrlModule] Robot {result[0]} has completed control. Robot_list is updated now ({user_data.robot_list})")

            # 로봇 상태 IDLE로 초기화
            updateRobotStatus(robotID=result[0], status=IDLE)
            rospy.logwarn(f"[CtrlModule] robot {result[0]}'s status updated to {IDLE}.")
        
        if len(user_data.robot_list) > 0:
            return True
        
        return False


class CtrlModuleSM(smach.StateMachine):
    def __init__(self, direction:String):
        smach.StateMachine.__init__(self, outcomes=["complete"],
                                    input_keys=['command', 'scene', 'robot_list'],
                                    output_keys=['robot_list'])
        with self:
            self.add('CONTROL_REQUEST', ControlRequest(direction=direction),
                     transitions={'done': 'IN_CONTROL',
                                  'none': 'complete'})
            self.add('IN_CONTROL', InControl(),
                     transitions={'invalid': 'complete',
                                'valid': 'IN_CONTROL',
                                'preempted':'IN_CONTROL'})
