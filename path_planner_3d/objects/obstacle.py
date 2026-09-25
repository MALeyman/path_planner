# objects/obstacle.py
import numpy as np
from pyqtgraph.Qt import QtCore, QtWidgets
import pyqtgraph.opengl as gl
from pyqtgraph.Qt import QtGui
from .sphere_object import SphereObject


# class Obstacle(SphereObject):
# 	def __init__(self, id_obstacle, position, radius, view, space_size, speed=0.05):
# 		super().__init__(position, radius, (0, 1, 0, 1), view)  # зеленый
# 		self.space_size = space_size
# 		self.speed = speed
# 		self.id_obstacle = id_obstacle
# 		# Случайное начальное направление
# 		direction = np.random.uniform(-1, 1, 3)
# 		norm = np.linalg.norm(direction)
# 		self.direction = direction / norm if norm > 1e-8 else np.array([1,0,0])

# 	# def move(self):
# 	# 	new_pos = self.position + self.direction * self.speed
# 	# 	for i in range(3):
# 	# 		# Отражение от границ пространства
# 	# 		if new_pos[i] < 0 or new_pos[i] > self.space_size[i]:
# 	# 			self.direction[i] = -self.direction[i]
# 	# 			new_pos[i] = np.clip(new_pos[i], 0, self.space_size[i])
# 	# 	self.set_position(new_pos)







class Obstacle(SphereObject):
    """Препятствие — зелёная сфера."""

    def __init__(self, id_obstacle, position, radius, view, space_size, speed=0.05):
        super().__init__(position, radius, (0.0, 1.0, 0.0, 1.0), view, speed)
        self.id = int(id_obstacle)
        self.space_size = np.asarray(space_size, dtype=float)

        direction = np.random.uniform(-1.0, 1.0, 3)
        n = np.linalg.norm(direction)
        self.direction = direction / n if n > 1e-8 else np.array([1.0, 0.0, 0.0])

    # Используется только в physics-процессе (не для GUI)
    def to_dict(self):
        return {
            "id": self.id,
            "pos": self.position.copy(),
            "radius": self.radius,
            "direction": self.direction.copy(),
            "speed": self.speed,
        }