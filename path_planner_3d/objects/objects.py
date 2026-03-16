import numpy as np
from pyqtgraph.Qt import QtCore, QtWidgets
import pyqtgraph.opengl as gl
from pyqtgraph.Qt import QtGui


class SphereObject:
	def __init__(self, position, radius, color, view, speed=0):
		self.position = np.array(position, dtype=float)
		self.radius = radius
		self.color = color
		self.mesh = self.create_sphere()
		self.mesh.translate(*self.position)
		view.addItem(self.mesh)
		self.speed = speed
		

	def create_sphere(self):
		meshdata = gl.MeshData.sphere(rows=20, cols=20)
		sphere = gl.GLMeshItem(meshdata=meshdata, smooth=True, color=self.color, shader='shaded', glOptions='opaque')
		sphere.scale(self.radius, self.radius, self.radius)
		return sphere

	def set_position(self, new_pos):
		diff = np.array(new_pos) - self.position
		self.position = np.array(new_pos)
		self.mesh.translate(*diff)

	def get_position(self):
		return self.position

	def set_color(self, color):
		self.color = color
		self.mesh.setColor(color)
		
	def set_speed(self, new_speed):
		self.speed = np.array(new_speed)
		
	def get_speed(self):
		return self.speed.copy()
