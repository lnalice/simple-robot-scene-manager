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

    updatedStatusInfo = [moduleState, robotID]
    query = (
        "UPDATE Robot "
        "SET moduleState = %s "
        "WHERE id = %s"
    )
    result = cur.execute(query, updatedStatusInfo)
    
    cnx.commit()
    cnx.close()

    return True

"""
UPDATE a robot's cmd_vel 
[todo] navigation 위치값 x,y,r로 수정해야 함
- seconds(이동 시간;초), linX(직진속도), angZ(각속도)
"""
def updateRobotVelocity (robotID, seconds, linX, angZ) -> bool:

    cnx = connect_to_mysql(mysql_config, attempts=3)
    cur = cnx.cursor(buffered=True)

    updatedStatusInfo:list = [seconds, linX, angZ, robotID]
    query = (
        "UPDATE Robot "
        "SET seconds = %s, linX = %s, angZ = %s "
        "WHERE id = %s"
    )
    cur.execute(query, updatedStatusInfo)

    cnx.commit()
    cnx.close()

    return True

"""
UPDATE a robot's status 
- IDLE / MOVE / MODULE / FAIL
"""
def updateRobotStatus (robotID, status) -> bool:

    cnx = connect_to_mysql(mysql_config, attempts=3)
    cur = cnx.cursor(buffered=True)

    updatedStatusInfo:list = [status, robotID]
    query = (
        "UPDATE Robot "
        "SET status = %s "
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

"""
GET a robot's IDs by role
"""
def selectRobotIDsByRole(role: str) -> tuple:

    cnx = connect_to_mysql(mysql_config, attempts=3)
    cur = cnx.cursor(buffered=True)

    query =  (
        "SELECT id FROM Robot "
        "WHERE role = %s"
    )
    cur.execute(query, [role])

    result = cur.fetchall()

    cnx.close()

    statusInfo: tuple = tuple(id[0] for id in result) # ex. ("id1", "id2")

    return statusInfo