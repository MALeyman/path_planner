import os
import sys
import numpy as np
from PyQt5 import QtWidgets, QtCore, QtGui
import pyqtgraph.opengl as gl
import logging

from pyqtgraph.opengl import GLMeshItem, MeshData

from OpenGL.GL import *


class Radar:
	def __init__(self, view, robot_pos, direction, scan_range=20, sector_angle=np.pi/4):
		self.view = view
		self.robot_pos = np.array(robot_pos)
		self.direction = np.array(direction)
		self.scan_range = scan_range
		self.sector_angle = sector_angle
		
		# 5 линий пирамиды (4 ребра + основание)
		self.edges = []
		colors = [(1,1,0,0.6), (1,1,0,0.6), (1,1,0,0.6), (1,1,0,0.6), (1,1,0,0.3)]
		for color in colors:
			edge = gl.GLLinePlotItem(color=color, width=1, antialias=True, glOptions='translucent')
			self.view.addItem(edge)
			self.edges.append(edge)
		


		self.sphere_surface = GLMeshItem(
			meshdata=MeshData(), 
			smooth=False, 
			color=(1, 1, 0, 0.2),  
			drawFaces=True, 
			drawEdges=False,
			glOptions='translucent'
			)


		self.view.opts['glOptions'] = 'translucent'
		glEnable(GL_BLEND)
		glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA)

		self.view.addItem(self.sphere_surface)

		self.update()
	
	def update(self, robot_pos=None, direction=None):
		"""Обновляет пирамиду радара"""
		if robot_pos is not None:
			self.robot_pos = np.array(robot_pos)
		if direction is not None:
			self.direction = np.array(direction)
		
		# ★★★ ВЫЧИСЛЯЕМ ВЫСОТУ ПИРАМИДЫ ПО СФЕРЕ ★★★
		phi_max = self.sector_angle * 1.2
		pyramid_height = self.scan_range * np.cos(phi_max / 2)  # Пересечение со сферой!

		# Вычисляем пирамиду
		base_center = self.robot_pos + pyramid_height * self.direction
		base_side = 2 * pyramid_height * np.tan(self.sector_angle / 2)
		base_vertices = self.create_perpendicular_square(base_center, self.direction, base_side)
		
		# Рёбра пирамиды
		for i, edge in enumerate(self.edges[:4]):
			next_i = (i + 1) % 4
			edge_points = np.array([self.robot_pos, base_vertices[i], base_vertices[next_i]])
			edge.setData(pos=edge_points, mode='line_strip')
		
		md = self.create_spherical_surface()
		self.sphere_surface.setMeshData(meshdata=md, color=(0,1,1,0.1))

		# sphere_points = self.create_spherical_base()
		# self.sphere_base.setData(pos=sphere_points, size=4, color=(1,1,0,0.15))

		# Основание
		# base_points = np.vstack([base_vertices, base_vertices[0]])
		# self.edges[4].setData(pos=base_points, mode='line_strip')
	
	def create_perpendicular_square(self, center, direction_move, side_length):
		""" квадрат перпендикулярно направлению"""
		direction_move = direction_move / np.linalg.norm(direction_move)
		
		arbitrary_vector = np.array([0, 0, 1])
		if abs(np.dot(arbitrary_vector, direction_move)) > 0.99:
			arbitrary_vector = np.array([1, 0, 0])
		
		v1 = np.cross(direction_move, arbitrary_vector)
		v1 = v1 / (np.linalg.norm(v1) + 1e-8)
		v2 = np.cross(direction_move, v1)
		v2 = v2 / np.linalg.norm(v2)
		
		half_side = side_length / 2
		return np.array([
			center + half_side * (-v1 - v2),
			center + half_side * ( v1 - v2),
			center + half_side * ( v1 + v2),
			center + half_side * (-v1 + v2)
		])
	

	# def scan_pyramid_for_obstacles(self, obstacles):
	# 	"""Возвращает препятствия ВНУТРИ пирамиды радара"""
	# 	obstacles_info = []
		
	# 	# Вычисляем пирамиду (
	# 	base_center = self.robot_pos + self.scan_range * self.direction
	# 	base_side = 2 * self.scan_range * np.tan(self.sector_angle / 2)
	# 	base_vertices = self.create_perpendicular_square(base_center, self.direction, base_side)
		
	# 	# Сканируем препятствия
	# 	for i, obs in enumerate(obstacles):
	# 		if self.is_sphere_inside_pyramid(obs.get_position(), obs.radius):
	# 			distance = np.linalg.norm(obs.get_position() - self.robot_pos)
	# 			direction_to_obs = (obs.get_position() - self.robot_pos) / distance
	# 			obstacles_info.append({'index': i, 'pos': obs.position, 'dist': distance})
		
	# 	return obstacles_info


	def is_sphere_inside_pyramid(self, obs_pos, obs_radius):
		"""сфера внутри пирамиды?"""
		# 1. Расстояние до вершины пирамиды
		dist = np.linalg.norm(obs_pos - self.robot_pos)
		if dist > self.scan_range + obs_radius:
			return False
		
		# 2. Проверяем 4 грани пирамиды
		base_center = self.robot_pos + self.scan_range * self.direction
		base_vertices = self.create_perpendicular_square(base_center, self.direction, 
													2 * self.scan_range * np.tan(self.sector_angle / 2))
		
		# Нормали граней (apex=robot_pos)
		for i in range(4):
			p1 = self.robot_pos
			p2 = base_vertices[i]
			p3 = base_vertices[(i+1)%4]
			
			v1 = p2 - p1
			v2 = p3 - p1
			normal = np.cross(v1, v2)
			normal = normal / np.linalg.norm(normal)
			d = np.dot(normal, p1)
			
			dist_to_plane = np.dot(normal, obs_pos) - d
			if dist_to_plane < -obs_radius:  # Слишком далеко от грани
				return False
		
		return True  # Внутри всех граней



	def create_spherical_surface(self):
		"""Создаёт сетку сферической поверхности"""
		# Параметры сетки
		phi_steps = 15  # Широта
		theta_steps = 30  # Долгота
			
		phi_max = self.sector_angle * 1.3 # +30% больше угла пирамиды
	
		phi = np.linspace(0, phi_max/2, phi_steps)
		theta = np.linspace(0, 2*np.pi, theta_steps)
		PHI, THETA = np.meshgrid(phi, theta)
		
		# Сферические координаты (повёрнутые по направлению радара)
		x = self.scan_range * np.sin(PHI) * np.cos(THETA)
		y = self.scan_range * np.sin(PHI) * np.sin(THETA)
		z = self.scan_range * np.cos(PHI)
		
		# Поворот по направлению робота
		verts = self.rotate_points(np.column_stack([x.ravel(), y.ravel(), z.ravel()]), self.direction)
		verts += self.robot_pos  # Смещение к роботу
		
		# Вершины
		md = MeshData()
		md.setVertexes(verts.reshape(phi_steps, theta_steps, 3).reshape(-1, 3))
		
		# Грани (треугольники)
		faces = []
		for i in range(phi_steps-1):
			for j in range(theta_steps-1):
				v1 = i*theta_steps + j
				v2 = i*theta_steps + (j+1)
				v3 = (i+1)*theta_steps + j
				v4 = (i+1)*theta_steps + (j+1)
				
				faces.append([v1, v2, v3])
				faces.append([v2, v4, v3])
		
		md.setFaces(np.array(faces))
		return md

	def rotate_points(self, points, direction):
		"""Поворачивает точки по направлению радара"""
		# Упрощённый поворот (полная матрица позже)
		rot_matrix = self.direction_to_matrix(direction)
		return np.dot(points, rot_matrix.T)




	def direction_to_matrix(self, direction):
		"""Безопасная матрица поворота"""
		direction = np.array(direction)
		norm = np.linalg.norm(direction)
		if norm == 0:  # ★ Защита от нуля!
			direction = np.array([1, 0, 0])
		else:
			direction = direction / norm
		
		# Создаём базис
		up = np.array([0, 0, 1])
		if abs(np.dot(up, direction)) > 0.99:
			up = np.array([0, 1, 0])
		
		x = np.cross(up, direction)
		x_norm = np.linalg.norm(x)
		if x_norm == 0:  # ★ Защита!
			x = np.array([1, 0, 0])
		else:
			x = x / x_norm
		
		y = np.cross(direction, x)
		
		return np.column_stack([x, y, direction])

