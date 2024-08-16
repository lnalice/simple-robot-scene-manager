from collections import deque
import json

json_vel_loc = '/home/domain/robot_ws/src/scene_manager/src/data/scene_param_vel.json'

def getMoveFLow(scene:str, robot_list:list) -> deque:
    
    move_flow = deque()
            
    # read json file
    with open(json_vel_loc) as json_file:
            json_data = json.load(json_file)

            for robot_goal in json_data[scene]:
                id = robot_goal["robot_id"]

                if id not in robot_list:
                     continue
                
                sec = robot_goal["seconds"]
                lin_vel = robot_goal["lin_vel"]
                ang_vel = robot_goal["ang_vel"]
                delay_sec = robot_goal["move_delay"]

                task = "%s %d %f %f %f %f %f %f %d" %(id, sec, lin_vel["x"], lin_vel["y"], lin_vel["z"], 
                                                ang_vel["x"], ang_vel["y"], ang_vel["z"], delay_sec)
                move_flow.append(task)

    return move_flow

def getCtrlFlow(scene:str, robot_list:list) -> deque:
    
    ctrl_flow = deque()

    # read json file
    with open(json_vel_loc) as json_file:
            json_data = json.load(json_file)
            
            for robot_goal in json_data[scene]:
                id = robot_goal["robot_id"]

                if id not in robot_list:
                     continue
                
                module = robot_goal["module"]

                if float(module) == 0:
                     continue
                
                delay_sec = robot_goal["ctrl_delay"]
                
                task = "%s %f %d" %(id, module, delay_sec)

                ctrl_flow.append(task)

    return ctrl_flow


"""
** getOppositeMoveFLow **
this SELECT function is for 'go home'
- return opposit direction from scene
- return non-delay seconds
"""
def getOppositeMoveFLow(scene:str, robot_list:list) -> deque:
    
    move_flow = deque()

    # read json file
    with open(json_vel_loc) as json_file:
            json_data = json.load(json_file)
            
            for robot_goal in json_data[scene]:
                id = robot_goal["robot_id"]

                if id not in robot_list:
                     continue
                
                sec = robot_goal["seconds"]
                lin_vel = robot_goal["lin_vel"]
                ang_vel = robot_goal["ang_vel"]
                delay_sec = robot_goal["move_delay"]

                task = "%s %d %f %f %f %f %f %f %d" %(id, sec, -float(lin_vel["x"]), -float(lin_vel["y"]), -float(lin_vel["z"]), 
                                                float(ang_vel["x"]), float(ang_vel["y"]), float(ang_vel["z"]), 0)
                move_flow.append(task)

    return move_flow