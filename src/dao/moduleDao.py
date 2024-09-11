from collections import deque
from dao.db.connection import connect_to_mysql
from dao.db.config import mysql_config

def selectModuleDataByScene (scene: str, isOpposite: bool, robot_list: list) -> deque:
    cnx = connect_to_mysql(mysql_config, attempts=3)
    result = cnx.cursor(buffered=True)
    
    ctrl_flow = deque()

    query = (
        "SELECT robotID, degZ, degX, delay FROM SceneModule "
        "WHERE sceneID = %s"
    )
    result.execute(query, [scene])

    for (robotID, degZ, degX, delay) in result:
        task =""

        if robotID not in robot_list:
            continue

        if degX == 0 and degZ == 0:
            continue

        if isOpposite:
            task = "%s %f %f %d" %(robotID, -float(degZ), -float(degX), 0)
        else:
            task = "%s %f %f %d" %(robotID, degZ, degX, delay)
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