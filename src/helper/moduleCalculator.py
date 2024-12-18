_steps = [0, 1/4, 2/4, 3/4, 1]
_degZ = [0, -172.6, -345, -517.6, -1380]
_degX = [0, 162.5, 323, 485.5, 650]

def moduleState2deg(moduleState: float) -> tuple:
    
    target_idx = max(0, min(int(moduleState * 4), 4))

    return (-_degZ[target_idx], -_degX[target_idx])

def deg2moduleState(degZ: float, degX: float) -> float:
    target_idx = 0

    for dz in _degZ:
        if abs(float(dz)) >= abs(degZ):
            break
        target_idx +=1
    
    if degZ > 0:
        return -_steps[target_idx]
    
    return _steps[target_idx]