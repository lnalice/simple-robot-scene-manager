from collections import deque
import json

json_vel_loc = '/home/domain/robot_ws/src/scene_manager/src/data/scene_param_vel.json'
"""
** uiTopicInterpreter **
TOPIC FORMAT
- SCENE     {scene ID}
- MOVE      {action; goal or home} {scene ID} {robot names}
- MODULE    {scene ID} {robot names}
"""
def getOppositeMoveFLow(scene:str) -> deque:
    
    move_flow = deque()

    # read json file
    with open(json_vel_loc) as json_file:
            json_data = json.load(json_file)
            
            for robot_goal in json_data[scene]:
                id = robot_goal["robot_id"]
                sec = robot_goal["seconds"]
                lin_vel = robot_goal["lin_vel"]
                ang_vel = robot_goal["ang_vel"]
                delay_sec = robot_goal["move_delay"]

                task = "%s %d %f %f %f %f %f %f %d" %(id, sec, -float(lin_vel["x"]), -float(lin_vel["y"]), -float(lin_vel["z"]), 
                                                float(ang_vel["x"]), float(ang_vel["y"]), float(ang_vel["z"]), 0)
                move_flow.append(task)

    return move_flow