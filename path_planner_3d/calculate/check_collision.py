# calculate/check_collision.py
import numpy as np



# def check_collision(obj1, obj2):
# 	"""БЕЗОПАСНАЯ проверка столкновений"""
# 	try:                    # 
# 		diff = obj1.position - obj2.position      #
# 		dist_sq = diff[0]**2 + diff[1]**2 + diff[2]**2  #  
# 		dist = np.sqrt(dist_sq)                    # 
# 		return dist < (obj1.radius + obj2.radius)  # 
# 	except:                  # 
# 		return False         # 



def check_collision(obj1, obj2) -> bool:
	try:  
		"""Проверка столкновения двух сферических объектов."""
		diff = np.asarray(obj1.position, dtype=float) - np.asarray(obj2.position, dtype=float)
		dist = np.sqrt(np.dot(diff, diff))
		return bool(dist < (obj1.radius + obj2.radius))
	except:                  
		return False         
