from collections import deque
import json
from typing import Tuple

json_vel_loc = '/home/domain/robot_ws/src/scene_manager/src/data/scene_param_vel.json'

def getMoveFLow(scene:str) -> deque:
    
    move_flow = deque()

    # read json file
    with open(json_vel_loc) as json_file:
            json_data = json.load(json_file)
            
            for robot_goal in json_data[scene]:
                id = robot_goal["robot_id"]
                sec = robot_goal["seconds"]
                lin_vel = robot_goal["lin_vel"]
                ang_vel = robot_goal["ang_vel"]

                # move_flow.append({"robot_id": id, 
                #                   "lin_x": lin_vel["x"], "lin_y": lin_vel["y"], "lin_z": lin_vel["z"],
                #                   "ang_x": ang_vel["x"], "ang_y": ang_vel["y"], "ang_z": ang_vel["z"]})

                task = "%s %d %f %f %f %f %f %f" %(id, sec, lin_vel["x"], lin_vel["y"], lin_vel["z"], 
                                                ang_vel["x"], ang_vel["y"], ang_vel["z"])
                move_flow.append(task)

    return move_flow

def getCtrlFlow(scene:str) -> deque:
    
    ctrl_flow = deque()

    # read json file
    with open(json_vel_loc) as json_file:
            json_data = json.load(json_file)
            
            for robot_goal in json_data[scene]:
                id = robot_goal["robot_id"]
                module = robot_goal["module"]
                ctrl_flow.append({"robot_id": id, "module": module})

    return ctrl_flow