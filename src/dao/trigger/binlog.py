import rospy

from pymysqlreplication import BinLogStreamReader
from pymysqlreplication.row_event import UpdateRowsEvent

from db.config import bitlog_config

"""
*Robot Status UPDATE 이벤트 감지
- IDLE 로봇 집합 업데이트
"""
def robot_event_cb(idle_robots: set, event: UpdateRowsEvent) -> bool:

    if not isinstance(event, UpdateRowsEvent): # event 타입 확인
        return False
    
    for row in event.rows: # 이벤트 값 확인

        # before_values = row["before_values"]
        after_values = row["after_values"]

        if "staus" not in after_values: # status 값 변화 확인
            return False
        
        cur_status = after_values["status"]
        robot = after_values["id"]

        if cur_status == "IDLE": # IDLE 로봇 업데이트
            idle_robots.add(robot)
        else:
            idle_robots.discard(robot)
        
        rospy.loginfo(f"Robot {robot}'s status is updated to {cur_status}.")

        return True

"""
*BitLogStreamReader 설정
"""
bitlog_stream = BinLogStreamReader(
        connection_settings = bitlog_config,
        only_tables=['Robot'],
        only_events=[robot_event_cb]
)