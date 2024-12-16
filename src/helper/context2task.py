from typing import Dict, List

from dao.contextMoveDao import selectMoveDataByContextID
from dao.contextModuleDao import selectModuleDataByContextID
from dao.moduleDao import selectDegreeByState
from dao.RobotDao import selectModuleStateByRobotID

"""
context와 배정된 로봇들을 가지고 실제 데이터를 불러와 task로 반환
@oaram  contextID       ex. FRIEND
@param  robotsByRole    ex. {"table": ("bot1", "bot2"), "chair": ("bot3")}
@output task            ex. ["MOVE bot1 {data}", "MODULE bot3 {data}", "MOVE bot2 {data}"] 
"""
def context2task(contextID: str, robotsByRole: Dict[str, List[str]]): 
    move_data = selectMoveDataByContextID(contextID)
    module_data = selectModuleDataByContextID(contextID)
    
    combined_data = move_data + module_data
    
    combined_data.sort(key=lambda x: x[-1]) # order 기준으로 정렬

    task = []
    robotsByData = {} # ex. {"table 1": "tb3_1", "table 2": "tb3_3"}
    
    for row in combined_data:
        role, roleID = row[:2]
        data = row[2:-1] # 마지막 칼럼 'order' 제외한 데이터
        
        # 1. 역할 별 로봇 실제 데이터 할당
        robot = robotsByData.get(f"{role} {roleID}")

        if not robot:
            robot = robotsByRole[role][roleID - 1]
            robotsByData[f"{role} {roleID}"] = robot

        # 2-1. [MODULE] 모듈 state -> 모터 각도차
        if len(data) <= 2:
            state, delay = data
            degZ, degX = selectDegreeByState(state=state) # 목표 모듈 확장정도를 위한 각도 변위

            (cur_state,) = selectModuleStateByRobotID(robotID=robot) # 현재 모듈 확장정도

            cur_degZ, cur_degX = selectDegreeByState(state=cur_state) # 현재 모듈 확장정도를 위한 각도 변위

            # 목표 각도 변위
            new_degZ = degZ - cur_degZ
            new_degX = degX - cur_degX

            data = f"{new_degZ} {new_degX} {delay}"
            task.append(f"MODULE {robot} {data}")
        
        # 2-2. [MOVE]
        else:
            data = ' '.join(map(str, data))
            task.append(f"MOVE {robot} {data}")
                     
    return task