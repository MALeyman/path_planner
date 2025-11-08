
import os
import logging
import numpy as np
import sys
from pyqtgraph.Qt import QtCore, QtWidgets
import pyqtgraph.opengl as gl
from pyqtgraph.Qt import QtGui
from objects import Robot, Obstacle
from calculate.check_collision import check_collision


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





class MainWindow(QtWidgets.QWidget):
    def __init__(self, space_size=(100,100,100), num_obstacles=10, start_pos=None, goal_pos=None, speed_robot=1.0, size_obstacles=2, radar_distance=20):
        super().__init__()
        self.setWindowTitle("3D Robot Animation with Control")

        #  ------------------     Параметры   -------------------
        self.space_size = space_size
        self.num_obstacles = num_obstacles
        self.start_pos = start_pos if start_pos is not None else [self.space_size[0]/4, self.space_size[1]/4, self.space_size[2]/4]
        self.goal_pos = goal_pos if goal_pos is not None else [0, 0, 0]
        self.speed_robot = speed_robot
        self.size_obstacles = size_obstacles
        self.radar_distance=radar_distance

        # -------------------    Интерфейс   ----------------------
        self.view = gl.GLViewWidget()
        # self.view.setCameraPosition(distance=180)
        self.setup_camera()

        self.start_button = QtWidgets.QPushButton("Старт")
        self.pause_button = QtWidgets.QPushButton("Пауза")
        self.stop_button = QtWidgets.QPushButton("Стоп")

        layout = QtWidgets.QVBoxLayout()
        layout.addWidget(self.view)
        btn_layout = QtWidgets.QHBoxLayout()
        btn_layout.addWidget(self.start_button)
        btn_layout.addWidget(self.pause_button)
        btn_layout.addWidget(self.stop_button)
        layout.addLayout(btn_layout)
        self.setLayout(layout)

        # -----------------   Таймер для анимации   --------------------
        self.timer = QtCore.QTimer()
        self.timer.timeout.connect(self.update_animation)

        # -----------------   Кнопки   ---------------------
        self.start_button.clicked.connect(self.start_animation)
        self.pause_button.clicked.connect(self.pause_animation)
        self.stop_button.clicked.connect(self.stop_animation)

        # Инициализация визуализации
        self.init_visualizer()

    def init_visualizer(self):
        #  ----------------------------  Сетка, оси  ---------------------------------------
        self.grid = ShiftedGridItem(size=max(self.space_size), spacing=10, color=(0.5,0.5,0.5,0.7))
        self.grid.add_to_view(self.view)
        self.add_axes()

        # ----------------------------     Цель     ----------------------------------------
        self.goal_scatter = gl.GLScatterPlotItem(
            pos=np.array([self.goal_pos]),
            size=10,
            color=(1,0,0,1),
            pxMode=True
        )
        self.view.addItem(self.goal_scatter)

        # -----------------------------    Робот и препятствия  -----------------------------
        self.robot = Robot(self.start_pos, radius=1.0, view=self.view)
        self.obstacles = []
        for _ in range(self.num_obstacles):
            pos = np.random.uniform(0, min(self.space_size), 3)
            obstacle = Obstacle(pos, radius=self.size_obstacles, view=self.view, space_size=self.space_size)
            self.obstacles.append(obstacle)

        # ---------------------------     Линии пути    ------------------------------------
        self.path_positions = [self.robot.position.copy()]
        self.path_line = gl.GLLinePlotItem(
            pos=np.array(self.path_positions),
            color=(1,1,0,1),
            width=1,
            antialias=True
        )
        self.view.addItem(self.path_line)

        # --------------------------     Линия к цели    ----------------------------------
        self.direct_line = gl.GLLinePlotItem(
            pos=np.array([self.robot.position, self.goal_pos]),
            color=(1,1,1,0.5),
            width=0.5,
            antialias=True,
            mode='line_strip'
        )
        self.view.addItem(self.direct_line)

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
        logging.info("Начало движения")
        self.timer.start(30)

    def pause_animation(self):
        logging.info("Пауза анимации")
        self.timer.stop()

    def stop_animation(self):
        logging.info("Стоп анимации")
        self.timer.stop()
        # Здесь можно сбросить состояние робота, пути, если потребуется

    def update_animation(self):
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



        for obs in self.obstacles:
            obs.move()
            dist = np.linalg.norm(obs.get_position() - self.robot.get_position())
            if dist < self.radar_distance:
                obs.set_color((0.6, 1.0, 0.6, 1))  # светло-зеленый RGBA
            else:
                obs.set_color((0, 1, 0, 1))        # обычный зеленый

            if check_collision(self.robot, obs):
                obs.set_color((1, 0, 0, 1))
                logging.info("Столкновение с препятствием!")
                self.timer.stop()
                return



    def setup_camera(self):
        ''' 
        Установка камеры
        '''

        pos_x = self.space_size[0] * 0.5
        pos_y = self.space_size[1] * 0.5
        pos_z = self.space_size[2] * 0.5
        self.view.setCameraPosition(pos=QtGui.QVector3D(pos_x, pos_y, pos_z), distance=300,  elevation=10, azimuth=-90)

