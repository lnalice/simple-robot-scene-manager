from collections import deque
import json
from typing import Tuple


json_loc = '/home/domain/robot_ws/src/scene_manager/src/data/scene_param.json'

def getSceneFlow(scene:str) -> Tuple[deque, deque]:
    
    move_flow = deque()
    ctrl_flow = deque()

    # read json file
    with open(json_loc) as json_file:
            json_data = json.load(json_file)
            
            for robot_goal in json_data[scene]:
                id = robot_goal["robot_id"]
                position = robot_goal["position"]
                orientation = robot_goal["orientation"]
                module = robot_goal["module"]

                move_task = "%s %f %f %f %f" %(id, position["x"], position["y"], orientation["z"], orientation["w"])
                ctrl_task = "%s %f" %(id, module)

                move_flow.append(move_task)
                ctrl_flow.append(ctrl_task)

    return move_flow, ctrl_flow

def getMoveFLow(scene:str) -> deque:
    
    move_flow = deque()

    # read json file
    with open(json_loc) as json_file:
            json_data = json.load(json_file)
            
            for robot_goal in json_data[scene]:
                id = robot_goal["robot_id"]
                position = robot_goal["position"]
                orientation = robot_goal["orientation"]

                task = "%s %f %f %f %f" %(id, position["x"], position["y"], orientation["z"], orientation["w"])

                move_flow.append(task)

    return move_flow

def getCtrlFlow(scene:str) -> deque:
    
    ctrl_flow = deque()

    # read json file
    with open(json_loc) as json_file:
            json_data = json.load(json_file)
            
            for robot_goal in json_data[scene]:
                id = robot_goal["robot_id"]
                module = robot_goal["module"]

                task = "%s %f" %(id, module)

                ctrl_flow.append(task)

    return ctrl_flow