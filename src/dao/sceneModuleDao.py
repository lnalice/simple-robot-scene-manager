from collections import deque
from dao.db.connection import connect_to_mysql
from dao.db.config import mysql_config

from dao.moduleDao import selectDegreeByState
from dao.RobotDao import selectModuleStateByRobotID

def selectModuleDataByScene (scene: str, isOpposite: bool, robot_list: list) -> deque:
    cnx = connect_to_mysql(mysql_config, attempts=3)
    result = cnx.cursor(buffered=True)
    
    ctrl_flow = deque()
    
    print(f"scene is {scene}.")
    query = (
        "SELECT robotID, state, delay FROM SceneModule "
        "WHERE sceneID = %s"
    )
    result.execute(query, [scene])

    for (robotID, state, delay) in result:
        task =""

        if robotID not in robot_list:
            continue

        degZ, degX = selectDegreeByState(state=state) # 목표 모듈 확장정도를 위한 각도 변위

        (cur_state,) = selectModuleStateByRobotID(robotID=robotID) # 현재 모듈 확장정도

        cur_degZ, cur_degX = selectDegreeByState(state=cur_state) # 현재 모듈 확장정도를 위한 각도 변위

        # 목표 각도 변위
        new_degZ = degZ - cur_degZ
        new_degX = degX - cur_degX

        if new_degX == 0 and new_degZ == 0:
            continue

        if isOpposite:
            task = "%s %f %f %d" %(robotID, -float(new_degZ), -float(new_degX), 0)
        else:
            task = "%s %f %f %d" %(robotID, new_degZ, new_degX, delay)
        
        ctrl_flow.append(task)

    cnx.close()

    return ctrl_flow

if __name__ == "__main__":
    ctrl_flow:deque = selectModuleDataByScene("scene_1", False)

    i:int = 0
    while ctrl_flow:
            
            goal_data = ctrl_flow.popleft() 

            print("goal_data [%d] %s" % (i, goal_data))
            i += 1
