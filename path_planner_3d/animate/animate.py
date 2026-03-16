
import os
import logging
import numpy as np
import sys
import os
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from pyqtgraph.Qt import QtCore, QtWidgets
import pyqtgraph.opengl as gl
from pyqtgraph.Qt import QtGui
from objects import Robot, Obstacle
from calculate.check_collision import check_collision
from radar.radar import Radar
import pyqtgraph as pg 
import multiprocessing as mp
from queue import Empty, Queue

# from animate.distance_matrix import DistanceMatrixWindow
from .distance_matrix import DistanceMatrixWindow, MatrixDistance

red = QtGui.QColor(255, 0, 0)
green = QtGui.QColor(0, 255, 0)
blue = QtGui.QColor(0, 0, 255)




class ShiftedGridItem:
	''' 
	Сетка
	'''
	def __init__(self, size=10, spacing=10, color=(0.5, 0.5, 0.5, 1)):
		self.size = size
		self.spacing = spacing
		self.color = color
		self.lines = []

	def add_to_view(self, view):
		for i in np.arange(0, self.size + self.spacing, self.spacing):
			# Линии параллельные X
			pts_x = np.array([[0, i, 0], [self.size, i, 0]])
			line_x = gl.GLLinePlotItem(pos=pts_x, color=self.color, width=1)
			view.addItem(line_x)
			self.lines.append(line_x)
			# Линии параллельные Y
			pts_y = np.array([[i, 0, 0], [i, self.size, 0]])
			line_y = gl.GLLinePlotItem(pos=pts_y, color=self.color, width=1)
			view.addItem(line_y)
			self.lines.append(line_y)


# Главное окно приложения
class MainWindow(QtWidgets.QWidget):
	def __init__(self, space_size=(100,100,100), num_obstacles=10, start_pos=None, goal_pos=None, speed_robot=1.0, size_obstacles=2, radar_distance=20, sector_angle = np.pi/4, speed_obstacles=0, matrix_size=91, startup_test=True):
		super().__init__()
		self.setWindowTitle("3D Планировщик")

		#  ------------------     Параметры   -------------------
		self.space_size = space_size
		self.num_obstacles = num_obstacles
		self.start_pos = start_pos if start_pos is not None else [self.space_size[0]/4, self.space_size[1]/4, self.space_size[2]/4]
		self.goal_pos = goal_pos if goal_pos is not None else [0, 0, 0]
		self.speed_robot = speed_robot
		self.size_obstacles = size_obstacles
		self.radar_distance = radar_distance
		self.sector_angle = sector_angle
		self.speed_obstacles = speed_obstacles
		self.detected_obstacles = []
		self.matrix_size = matrix_size
		self.startup_test =startup_test

		# -------------------    Интерфейс   ----------------------

		# -------------------    СОЗДАЁМ ВСЕ ВИДЖЕТЫ 
		self.view = gl.GLViewWidget()
		# self.view.setCameraPosition(distance=180)
		self.view.renderOrder = 'frontToBack' 
		self.setup_camera()


		# ★★★ ВТОРОЕ ОКНО ★★★
		self.distance_window = DistanceMatrixWindow(matrix_size=self.matrix_size, radar_distance=self.radar_distance)
		self.distance_window.show()
		
		# Матрица дистанций
		self.matrix_calc = MatrixDistance(matrix_size=self.matrix_size, scan_range=self.radar_distance, num_sectors=self.matrix_size)
		self.matrix_counter = 0


		self.start_button = QtWidgets.QPushButton("Старт")
		self.pause_button = QtWidgets.QPushButton("Пауза")
		self.stop_button = QtWidgets.QPushButton("Стоп")


		# Кнопки
		self.start_button = QtWidgets.QPushButton("Старт")
		self.pause_button = QtWidgets.QPushButton("Пауза")
		self.stop_button = QtWidgets.QPushButton("Стоп")

	
		# ============== главный  LAYOUT ===============
		main_layout = QtWidgets.QHBoxLayout()
		
		# ----------   Левая: 3D -------
		left_layout = QtWidgets.QVBoxLayout()
		left_layout.addWidget(self.view, stretch=1)
		btn_layout = QtWidgets.QHBoxLayout()
		btn_layout.addWidget(self.start_button)
		btn_layout.addWidget(self.pause_button)
		btn_layout.addWidget(self.stop_button)
		left_layout.addLayout(btn_layout)
		main_layout.addLayout(left_layout, stretch=3)


		layout = QtWidgets.QVBoxLayout()
		layout.addLayout(main_layout)
		self.setLayout(layout)


		# Явно задаём размер и позицию
		self.resize(1400, 900)           # Ширина × высота
		self.move(50, 50)                # Позиция на экране
		self.setMinimumSize(1000, 700)   # Минимальный размер


		# ===========   Таймер для анимации   ======
		self.timer = QtCore.QTimer()
		#  Соединение сигнала со слотом: 
		# self.timer.timeout - (Сигнал по тайм ауту) ──> self.update_animation() - (Слот)
		self.timer.timeout.connect(self.update_animation)

		# ===========   Кнопки   ========
		# start_button.clicked  ──(сигнал)──>  start_animation()- (Слот)
		self.start_button.clicked.connect(self.start_animation)
		# pause_button.clicked  ──(сигнал)──>  pause_animation()- (Слот)
		self.pause_button.clicked.connect(self.pause_animation)
		# stop_button.clicked  ──(сигнал)──>  stop_animation()- (Слот)
		self.stop_button.clicked.connect(self.stop_animation)


		# Инициализация визуализации
		self.init_visualizer()


	def init_visualizer(self):
		'''
			Инициализация визуализации
		'''
		#  ----------------------------  Сетка, оси  ---------------------------------------
		self.grid = ShiftedGridItem(size=max(self.space_size), spacing=10, color=(0.5,0.5,0.5,0.7))
		self.grid.add_to_view(self.view)
		self.add_axes()

		# ----------------------     Цель     --------------------------------
		self.goal_scatter = gl.GLScatterPlotItem(
			pos=np.array([self.goal_pos]),
			size=10,
			color=(1,0,0,1),
			pxMode=True
		)
		self.view.addItem(self.goal_scatter)

	
		# ----------------------   Робот и препятствия  -----------------------
		self.robot = Robot(
					self.start_pos, 
					radius=1.0, 
					view=self.view, 
					direction=(np.array(self.goal_pos) - np.array(self.start_pos)) / 
							np.linalg.norm(np.array(self.goal_pos) - np.array(self.start_pos)),
					speed=self.speed_robot
				)

		self.obstacles = []  # Список препятствий
		# self.num_obstacles = 2
		pos1 = [[10, 20 , 100], [20, 10, 100], [10, 10, 110], [10, 10, 90]]
		pos1 = [ [30, 0, 100]]
		# self.num_obstacles = len(pos1)
		print("Позиция всех препятствий = ", pos1)
		print("Стартовая позиция робота ==", self.start_pos)
		print("Позиция цели == ", self.goal_pos)
		for num_obstacle in range(self.num_obstacles):
			pos = np.random.uniform(0, min(self.space_size), 3)
			# pos = [10, -10 + num_obstacle * 30, self.space_size[2]/2]  
			# pos = pos1[num_obstacle]
			
			print("Позиция текущего препятствия = ", pos)
			obstacle = Obstacle(num_obstacle, pos, radius=self.size_obstacles, view=self.view, space_size=self.space_size, speed=self.speed_obstacles)
			self.obstacles.append(obstacle)


		# ---------------   РАДАР ---------------------
		self.radar = Radar(self.view, self.robot.position, self.robot.direction, 
						self.radar_distance, sector_angle=self.sector_angle)


		# ---------------------    Линии пути    -------------------------------
		self.path_positions = [self.robot.position.copy()]
		self.path_line = gl.GLLinePlotItem(
			pos=np.array(self.path_positions),
			color=(1,1,0,1),
			width=1,
			antialias=True
		)
		self.view.addItem(self.path_line)

		# -----------------       Линия к цели    ---------------------------
		self.direct_line = gl.GLLinePlotItem(
			pos=np.array([self.robot.position, self.goal_pos]),
			color=(1,1,1,0.5),
			width=0.5,
			antialias=True,
			mode='line_strip'
		)
		self.view.addItem(self.direct_line)

		self.update_animation()


	def add_axes(self):
		axis_length = max(self.space_size)*1.1
		# X
		x_axis = gl.GLLinePlotItem(pos=np.array([[0,0,0],[axis_length,0,0]]), color=QtGui.QColor(255,0,0), width=3, antialias=True)
		self.view.addItem(x_axis)
		# Y
		y_axis = gl.GLLinePlotItem(pos=np.array([[0,0,0],[0,axis_length,0]]), color=QtGui.QColor(0,255,0), width=3, antialias=True)
		self.view.addItem(y_axis)
		# Z
		z_axis = gl.GLLinePlotItem(pos=np.array([[0,0,0],[0,0,axis_length]]), color=QtGui.QColor(0,255,255), width=3, antialias=True)
		self.view.addItem(z_axis)


	def start_animation(self):
		"""Запуск анимации"""
		if not self.timer.isActive():
			logging.info("Начало движения")
			self.timer.start(33)  # 30мс = 33 FPS

	def pause_animation(self):
		logging.info("Пауза анимации")
		self.timer.stop()

	def stop_animation(self):
		# Здесь можно сбросить состояние робота, пути, если потребуется
		logging.info("Стоп анимации")
		self.timer.stop()
		
	def update_animation(self):
		# if not self.running:
		#     return
		desired_dir = self.goal_pos - self.robot.get_position()     # Направление на цель
		norm = np.linalg.norm(desired_dir)    # Расстояние до цели
		if norm < 0.6:
			logging.info("Цель достигнута!")
			self.timer.stop()
			return

		# ------------------------ Отрисовка линий   --------------------
		self.path_positions.append(self.robot.get_position().copy())
		self.path_line.setData(pos=np.array(self.path_positions))                                # Линия пути робота
		self.direct_line.setData(pos=np.array([self.robot.get_position(), self.goal_pos]))       # Линия направления на цель


		# ------------------------   Перемещение робота  -------------------
		desired_dir = desired_dir / norm
		self.robot.set_direction(desired_dir)  # Новое курсовое направление робота
		self.robot.move_step()


		# --------------------------  ОБНОВЛЯЕМ РАДАР  ------------
		self.radar.update(self.robot.position, self.robot.direction)
		
		# ------------------------- ОБНОВЛЕНИЕ ПРЕПЯТСТВИЙ --------
		self.detected_obstacles = [] # Список обнаруженных препятствий
		for obs in self.obstacles:
			obs.move()
			# dist = np.linalg.norm(obs.get_position() - self.robot.get_position())
			# Если препятствие попало в зону действия радара
			if self.radar.is_sphere_inside_pyramid(obs.position, obs.radius): 
				obs.set_color((1.0, 1.0, 0.0, 1.0))  # Жёлтый!
				self.detected_obstacles.append(obs)
			else:
				obs.set_color((0.0, 1.0, 0.0, 1.0))  # Зелёный!

			# Если столкновение
			if check_collision(self.robot, obs):
				obs.set_color((1, 0, 0, 1))
				logging.info("Столкновение с препятствием!")
				self.timer.stop()
				return

		# ---   Обновление  МАТРИЦЫ ДИСТАНЦИЙ ---
		matrix, min_dist, detected, angle = self.matrix_calc.update_distance_matrix(self.robot, self.detected_obstacles, self.startup_test)
		print(matrix)
		print(min_dist)
		print(detected)
		print(angle)

		# min_val = np.min(matrix)
		# idx_flat = np.argmin(matrix)

		# # 3. Перевести в пару индексов (i, j)
		# i, j = np.unravel_index(idx_flat, matrix.shape)
		# print("Минимальный элемент:", min_val)
		# print("Индексы (i, j):", i, j)

		# Курсовой вектор
		self.distance_window.set_robot_direction(angle, self.startup_test)

		# Визуализация матрицы дистанций
		self.distance_window.update_matrix(matrix)


	def setup_camera(self):
		''' 
		Установка камеры
		'''

		pos_x = self.space_size[0] * 0.5
		pos_y = self.space_size[1] * 0.5
		pos_z = self.space_size[2] * 0.5
		self.view.setCameraPosition(pos=QtGui.QVector3D(pos_x, pos_y, pos_z), distance=300,  elevation=10, azimuth=-90)


	def rotate_vector(self, vec, yaw, pitch):
		"""Поворот вектора на yaw(гориз), pitch(верт)"""
		# Матрицы поворота
		cy, sy = np.cos(yaw), np.sin(yaw)
		cp, sp = np.cos(pitch), np.sin(pitch)
		
		Rx = np.array([[1, 0, 0],
					[0, cp, -sp],
					[0, sp, cp]])
		
		Ry = np.array([[cy, 0, sy],
					[0, 1, 0],
					[-sy, 0, cy]])
		
		return Rx @ Ry @ vec
