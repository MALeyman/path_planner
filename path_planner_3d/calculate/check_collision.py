import numpy as np

def check_collision(obj1, obj2):
    dist = np.linalg.norm(obj1.position - obj2.position)
    if dist < (obj1.radius + obj2.radius):
        return True
    return False