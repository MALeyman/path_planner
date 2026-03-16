import numpy as np

# def check_collision(obj1, obj2):
#     '''
#     Расстояние между объектами
	
#     :param obj1: объект 1
#     :param obj2: Объект 2
#     '''

#     dist = np.linalg.norm(obj1.position - obj2.position) # Расстояние между центрами объектов
#     if dist < (obj1.radius + obj2.radius):    
#         return True
#     return False


def check_collision(obj1, obj2):
	"""БЕЗОПАСНАЯ проверка столкновений"""
	try:                    # ← 1 ТАБ
		diff = obj1.position - obj2.position      # ← 2 ТАБА
		dist_sq = diff[0]**2 + diff[1]**2 + diff[2]**2  # ← 2 ТАБА  
		dist = np.sqrt(dist_sq)                    # ← 2 ТАБА
		return dist < (obj1.radius + obj2.radius)  # ← 2 ТАБА
	except:                  # ← 1 ТАБ  
		return False         # ← 2 ТАБА
