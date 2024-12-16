from typing import List, Dict
import random

import rospy

from dao.RobotDao import selectRobotIDsByRole

"""
:param reqRoles: 각 role 별로 필요한 로봇 개수
                 ex) { 'table': 10, 'chair': 15 }
:result:         각 role 별 할당된 로봇 리스트
                 ex) { 'table': ['bot1', 'bot3'], 'chair': ['bot2'] } 혹은 { }
"""
def randomAllocation(reqRoles: Dict[str, int]) -> Dict[str, List[str]]:
    selectedRobots = {}

    for role, count in reqRoles.items():
        if count == 0: # 필요한 로봇 없을 경우
            continue

        allRobots = selectRobotIDsByRole(role)
        
        if len(allRobots) < count: # 필요한 로봇이 부족한 경우가 하나라도 있다면, 빈 딕셔너리 반환
            rospy.logwarn(f"There are no enough robots for role {role}. (Task Rejected)")
            return {}

        selectedRobots[role] = list(random.sample(allRobots, count))
    
    return selectedRobots
