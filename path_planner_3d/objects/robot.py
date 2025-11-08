import numpy as np
from pyqtgraph.Qt import QtCore, QtWidgets
import pyqtgraph.opengl as gl
from pyqtgraph.Qt import QtGui

from objects import SphereObject

class Robot(SphereObject):
    def __init__(self, position, radius, view, speed=0.1):
        super().__init__(position, radius, (0, 0, 1, 1), view)  # Синий цвет робота
        self.speed = speed
        self.direction = np.array([1.0, 0.0, 0.0])  # начальное направление

    def set_direction(self, direction):
        norm = np.linalg.norm(direction)
        if norm > 1e-8:
            self.direction = direction / norm

    def move_step(self):
        new_pos = self.position + self.direction * self.speed
        self.set_position(new_pos)



