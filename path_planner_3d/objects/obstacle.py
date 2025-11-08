import numpy as np
from pyqtgraph.Qt import QtCore, QtWidgets
import pyqtgraph.opengl as gl
from pyqtgraph.Qt import QtGui
from objects import SphereObject


# class Obstacle(SphereObject):
#     def __init__(self, position, radius, view):
#         super().__init__(position, radius, (0, 1, 0, 1), view)  # Зеленый цвет препятствия




class Obstacle(SphereObject):
    def __init__(self, position, radius, view, space_size, speed=0.05):
        super().__init__(position, radius, (0, 1, 0, 1), view)  # зеленый
        self.space_size = space_size
        self.speed = speed
        # Случайное начальное направление
        direction = np.random.uniform(-1, 1, 3)
        norm = np.linalg.norm(direction)
        self.direction = direction / norm if norm > 1e-8 else np.array([1,0,0])

    def move(self):
        new_pos = self.position + self.direction * self.speed
        for i in range(3):
            # Отражение от границ пространства
            if new_pos[i] < 0 or new_pos[i] > self.space_size[i]:
                self.direction[i] = -self.direction[i]
                new_pos[i] = np.clip(new_pos[i], 0, self.space_size[i])
        self.set_position(new_pos)

