# objects/robot.py
import numpy as np
from pyqtgraph.Qt import QtCore, QtWidgets
import pyqtgraph.opengl as gl
from pyqtgraph.Qt import QtGui

from .sphere_object import SphereObject



class Robot(SphereObject):
    """Робот — синяя сфера с направлением движения."""

    def __init__(self, position, radius, view, direction=None, speed=0.1):
        super().__init__(position, radius, (0.0, 0.0, 1.0, 1.0), view, speed)
        if direction is None:
            direction = np.array([1.0, 0.0, 0.0])
        self.direction = self._normalize(direction)

    @staticmethod
    def _normalize(v):
        v = np.asarray(v, dtype=float)
        n = np.linalg.norm(v)
        if n < 1e-8:
            return np.array([1.0, 0.0, 0.0])
        return v / n

    def set_direction(self, direction):
        self.direction = self._normalize(direction)

    def get_direction(self):
        return self.direction.copy()

    def get_velocity(self):
        return self.direction * self.speed
