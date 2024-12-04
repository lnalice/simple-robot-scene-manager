from collections import deque
from dao.db.connection import connect_to_mysql
from dao.db.config import mysql_config

"""
UPDATE a robot's status
- current module state: moduleState (4steps: 0, 0.25, 0.5, 0.75, 1)
- current location: displacementX, displacementZ
    (calculate the accumulated velocity value on the x-axis and z-axis)
"""
def updateRobotModuleState (robotID, moduleState) -> bool:

    cnx = connect_to_mysql(mysql_config, attempts=3)
    cur = cnx.cursor(buffered=True)

    updatedStatusInfo = [str(moduleState), str(robotID)]
    query = (
        "UPDATE Robot "
        "SET moduleState = %s "
        "WHERE id = %s"
    )
    result = cur.execute(query, updatedStatusInfo)
    
    cnx.commit()
    cnx.close()

    return True

def updateRobotDisplacement (robotID, displacementX, displacementZ) -> bool:

    cnx = connect_to_mysql(mysql_config, attempts=3)
    cur = cnx.cursor(buffered=True)

    updatedStatusInfo:list = [displacementX, displacementZ, str(robotID)]
    query = (
        "UPDATE Robot "
        "SET displacementX = %s, displacementZ = %s "
        "WHERE id = %s"
    )
    cur.execute(query, updatedStatusInfo)

    cnx.commit()
    cnx.close()

    return True

"""
GET a robot's status
"""
def selectModuleStateByRobotID(robotID: str) -> tuple:

    cnx = connect_to_mysql(mysql_config, attempts=3)
    cur = cnx.cursor(buffered=True)

    query =  (
        "SELECT moduleState FROM Robot "
        "WHERE id = %s"
    )
    cur.execute(query, [robotID])

    statusInfo:tuple = cur.fetchone()

    cnx.close()

    return statusInfo

def selectDisplacementByRobotID(robotID: str) -> tuple:
    cnx = connect_to_mysql(mysql_config, attempts=3)
    cur = cnx.cursor(buffered=True)

    query =  (
        "SELECT displacementX, displacementZ FROM Robot "
        "WHERE id = %s"
    )
    cur.execute(query, [robotID])

    statusInfo:tuple = cur.fetchone()

    cnx.close()

    return statusInfo