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

                move_flow.append({"robot_id": id, 
                                  "pos_x": position["x"], "pos_y": position["y"],
                                  "ort_z": orientation["z"], "ort_w": orientation["w"]})
                ctrl_flow.append({"robot_id": id, "module": module})

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

                move_flow.append({"robot_id": id, 
                                  "pos_x": position["x"], "pos_y": position["y"],
                                  "ort_z": orientation["z"], "ort_w": orientation["w"]})

    return move_flow

def getCtrlFlow(scene:str) -> deque:
    
    ctrl_flow = deque()

    # read json file
    with open(json_loc) as json_file:
            json_data = json.load(json_file)
            
            for robot_goal in json_data[scene]:
                id = robot_goal["robot_id"]
                module = robot_goal["module"]
                ctrl_flow.append({"robot_id": id, "module": module})

    return ctrl_flow