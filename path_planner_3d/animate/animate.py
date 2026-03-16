
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
	def __init__(self, space_size=(100,100,100), num_obstacles=10, start_pos=None, goal_pos=None, speed_robot=1.0, size_obstacles=2, radar_distance=20, sector_angle = np.pi/4):
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

		# -------------------    Интерфейс   ----------------------
		# -------------------    СОЗДАЁМ ВСЕ ВИДЖЕТЫ 
		self.view = gl.GLViewWidget()
		# self.view.setCameraPosition(distance=180)
		self.view.renderOrder = 'frontToBack' 
		self.setup_camera()

		self.start_button = QtWidgets.QPushButton("Старт")
		self.pause_button = QtWidgets.QPushButton("Пауза")
		self.stop_button = QtWidgets.QPushButton("Стоп")


		# ---------------------   МАТРИЦА ДИСТАНЦИЙ ПЕРЕД layout 
		# self.distance_widget = pg.GraphicsLayoutWidget()
		# self.distance_widget.setFixedSize(300, 300)
		# self.distance_plot = self.distance_widget.addPlot(title="Матрица дистанций радара")
		# self.distance_img = pg.ImageItem()
		# self.distance_plot.addItem(self.distance_img)
		# self.distance_plot.getViewBox().setAspectLocked(True)
		# self.distance_img.setLookupTable(pg.colormap.get('grey'))
		# ★★★ РУЧНОЙ СЕРЫЙ COLORMAP ★★★
		# pos = np.array([0.0, 1.0])
		# color = np.array([[0,0,0,1], [1,1,1,1]])  # 0=чёрный, 1=белый
		# cmap = pg.ColorMap(pos, color)
		# lut = cmap.getLookupTable()
		# self.distance_img.setLookupTable(lut)
		# self.distance_plot.invertY(True)             # Y сверху вниз (как MATLAB!)
		# self.distance_plot.showGrid(x=True, y=True)

		# Кнопки
		self.start_button = QtWidgets.QPushButton("Старт")
		self.pause_button = QtWidgets.QPushButton("Пауза")
		self.stop_button = QtWidgets.QPushButton("Стоп")

		# ★★★ КРИТИЧНО: инициализация состояний ★★★
		# self.running = False
		# self.frame_count = 0
		# self.test_counter = 0

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


		# --------------    Правая: Heatmap  -----------
		# right_layout = QtWidgets.QVBoxLayout()
		# right_layout.addWidget(self.distance_widget, stretch=1)
		# main_layout.addLayout(right_layout, stretch=1)


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

		# QtCore.QTimer.singleShot(100, self.init_distance_matrix) 

		# # ==== Многопроцессорный радар ====
		# self.radar_queue = Queue()
		# self.radar_process = None



		# Инициализация визуализации
		self.init_visualizer()


	# def start_radar_process(self):
	#     """Запуск heatmap в отдельном процессе"""
	#     self.radar_process = mp.Process(target=self.radar_window_process)
	#     self.radar_process.start()
		
	# def radar_window_process(self):
	#     """Отдельный процесс: Heatmap окно"""
	#     app = pg.mkQApp("Radar Heatmap")
		
	#     win = pg.GraphicsLayoutWidget(show=True)
	#     win.setWindowTitle("Матрица дистанций радара")
	#     plot = win.addPlot(title="Radar Distance Matrix")
	#     img = pg.ImageItem()
	#     plot.addItem(img)
	#     plot.invertY(True)
	#     plot.showGrid(x=True, y=True)
		
	#     num_sectors = 64
	#     while True:
	#         try:
	#             matrix = self.radar_queue.get(timeout=0.1)
	#             img.setImage(matrix.T)
	#         except Empty:
	#             # Заглушка
	#             test_matrix = np.random.rand(num_sectors, num_sectors) * 20
	#             img.setImage(test_matrix.T)
	


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
							np.linalg.norm(np.array(self.goal_pos) - np.array(self.start_pos))
				)

		self.obstacles = []
		for _ in range(self.num_obstacles):
			pos = np.random.uniform(0, min(self.space_size), 3)
			obstacle = Obstacle(pos, radius=self.size_obstacles, view=self.view, space_size=self.space_size)
			self.obstacles.append(obstacle)


		# # ============  HEATMAP ЛИНИИ И ТЕКСТ ========
		# self.target_line = pg.PlotDataItem(pen=pg.mkPen('r', width=3))
		# self.direction_line = pg.PlotDataItem(pen=pg.mkPen('b', width=3))
		# self.distance_plot.addItem(self.target_line)
		# self.distance_plot.addItem(self.direction_line)
		
		# self.speed_text = pg.TextItem(text="Скорость: 0.0", color='w', anchor=(0,0))
		# self.distance_plot.addItem(self.speed_text)



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




	# def init_distance_matrix(self):
	#     """Инициализация heatmap ПОСЛЕ запуска"""
	#     self.target_line = pg.PlotDataItem(pen=pg.mkPen('r', width=3))
	#     self.direction_line = pg.PlotDataItem(pen=pg.mkPen('b', width=3))
	#     self.speed_text = pg.TextItem(text="Скорость: 0.0", color='w')
		
	#     self.distance_plot.addItem(self.target_line)
	#     self.distance_plot.addItem(self.direction_line)
	#     self.distance_plot.addItem(self.speed_text)






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
			self.timer.start(100)  # 30мс = 33 FPS

			# # Запуск heatmap процесса через 0.5с
			# QtCore.QTimer.singleShot(500, self.start_radar_process)


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
		if norm < 0.1:
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


		# ----------  ОБНОВЛЯЕМ РАДАР  ------------
		self.radar.update(self.robot.position, self.robot.direction)
		
		for obs in self.obstacles:
			obs.move()
			dist = np.linalg.norm(obs.get_position() - self.robot.get_position())
			if self.radar.is_sphere_inside_pyramid(obs.position, obs.radius): 
				obs.set_color((1.0, 1.0, 0.0, 1.0))  # Жёлтый!
			else:
				obs.set_color((0.0, 1.0, 0.0, 1.0))  # Зелёный!

			if check_collision(self.robot, obs):
				obs.set_color((1, 0, 0, 1))
				logging.info("Столкновение с препятствием!")
				self.timer.stop()
				return


		# ★★★ Отправка данных в процесс ========
		# radar_data = self.get_distance_matrix()
		# self.radar_queue.put(radar_data)

		# ----------  ОБНОВЛЕНИЕ МАТРИЦЫ ДИСТАНЦИЙ ----------
		matrix_dist = self.get_distance_matrix()  # Реализуйте!
		# self.update_distance_matrix(matrix_dist)



	def get_distance_matrix(self):
		"""Матрица дистанций радара"""
		num_sectors = 64
		matrix = np.full((num_sectors, num_sectors), np.nan)
		
		try:
			# Сканируем препятствия
			radar_hits = self.radar.scan_pyramid_for_obstacles(self.obstacles)
			for hit in radar_hits:
				# Преобразуем 3D -> углы (заглушка)
				az = np.random.uniform(-np.pi, np.pi)
				el = np.random.uniform(-np.pi/2, np.pi/2)
				
				i = int((az + np.pi) / (2*np.pi) * num_sectors)
				j = int((el + np.pi/2) / np.pi * num_sectors)
				
				if 0 <= i < num_sectors and 0 <= j < num_sectors:
					matrix[i, j] = hit.get('dist', np.random.uniform(5, 15))
		except:
			pass  # Если радар не готов — пустая матрица
		
		return matrix




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
