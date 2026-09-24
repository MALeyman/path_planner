# animate/animate.py

import os
import logging
import numpy as np
import sys
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from pyqtgraph.Qt import QtCore, QtWidgets
from pyqtgraph.Qt import QtGui
import pyqtgraph.opengl as gl
from objects import Robot, Obstacle
from calculate.check_collision import check_collision
from animate.distance_matrix import DistanceMatrixWindow
import pyqtgraph as pg
import multiprocessing as mp


from queue import Empty, Queue
from environment import physics_process
from environment import planner_process
from radar import MatrixDistance
from radar import Radar
from radar.core import radar_process

# animate/animate.py

import os
import logging
import numpy as np
import sys
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from pyqtgraph.Qt import QtCore, QtWidgets
from pyqtgraph.Qt import QtGui
import pyqtgraph.opengl as gl
from objects import Robot, Obstacle
from calculate.check_collision import check_collision
from radar.radar import Radar
from animate.distance_matrix import DistanceMatrixWindow
import pyqtgraph as pg
import multiprocessing as mp


# Для отображения
red = QtGui.QColor(255, 0, 0)
green = QtGui.QColor(0, 255, 0)
blue = QtGui.QColor(0, 0, 255)


class ShiftedGridItem:
    ''' 
    Сетка
    '''
    def __init__(self, size=10, spacing=10, color=(0.5,0.5,0.5,1)):
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
    def __init__(self, space_size=(100,100,100), num_obstacles=10, start_pos=None, goal_pos=None, speed_robot=1.0, size_obstacles=2, radar_distance=20, sector_angle = np.pi/4, speed_obstacles=0, matrix_size=91, startup_test=True, fps=33):
        super().__init__()
        self.setWindowTitle("3D Планировщик")

        # ------------------------------------------ Параметры   -------------------
        self._init_parameters(space_size, num_obstacles, start_pos, goal_pos, speed_robot, size_obstacles, radar_distance, sector_angle, speed_obstacles, matrix_size, startup_test, fps)

        self._create_widgets()
        self._setup_layout()
        self._setup_signals()
        self._init_visualizer()
        self._setup_queues_and_processes()

        self.setup_camera()


    def _init_parameters(self, space_size, num_obstacles, start_pos, goal_pos, speed_robot, size_obstacles, radar_distance, sector_angle, speed_obstacles, matrix_size, startup_test, fps):
        ''' 
        Инициализация параметров
        '''
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
        self.startup_test = startup_test
        self.fps = fps

        #  Таймер для анимации  
        self.timer = QtCore.QTimer()


    def _create_widgets(self):
        ''' 
        Создание виджетов
        '''
        self.view = gl.GLViewWidget()
        self.view.renderOrder = 'frontToBack'

        # ---------- ВТОРОЕ ОКНО (радар‑матрица)
        self.distance_window = DistanceMatrixWindow(matrix_size=self.matrix_size, radar_distance=self.radar_distance)
        self.distance_window.show()

        # ------------- Кнопки
        self.start_button = QtWidgets.QPushButton("Старт")
        self.pause_button = QtWidgets.QPushButton("Пауза")
        self.stop_button = QtWidgets.QPushButton("Стоп")


    def _setup_queues_and_processes(self):
        '''
        Очереди между процессами
        MainWindow — только получает данные, ничего не вычисляет.
        '''
        self.queue_env = mp.Queue()         # physics → GUI + radar (состояние мира)
        self.queue_radar = mp.Queue()       # radar → GUI (матрица и angle)
        self.queue_planner = mp.Queue()     # planner → GUI (направление и скорость робота)

        self.pause_event_physics = mp.Event()
        self.pause_event_radar = mp.Event()
        # Процессы (запускаются при старте анимации)
        self.p_physics = None
        self.p_radar = None
        self.p_planner = None


    def _setup_layout(self):
        ''' 
        Компоновка виджетов
        '''
        # Левая часть: 3D‑сцена
        left_layout = QtWidgets.QVBoxLayout()
        left_layout.addWidget(self.view, stretch=1)

        btn_layout = QtWidgets.QHBoxLayout()
        btn_layout.addWidget(self.start_button)
        btn_layout.addWidget(self.pause_button)
        btn_layout.addWidget(self.stop_button)
        left_layout.addLayout(btn_layout)

        main_layout = QtWidgets.QHBoxLayout()
        main_layout.addLayout(left_layout, stretch=3)

        central_layout = QtWidgets.QVBoxLayout()
        central_layout.addLayout(main_layout)
        self.setLayout(central_layout)

        # Явно задаём размер и позицию
        self.resize(1400, 900)           # Ширина × высота
        self.move(50, 50)                # Позиция на экране
        self.setMinimumSize(1000, 700)   # Минимальный размер


    def _setup_signals(self):
        ''' 
        Сигналы
        '''
        # Соединение кнопок с методами
        self.start_button.clicked.connect(self.start_animation)
        self.pause_button.clicked.connect(self.pause_animation)
        self.stop_button.clicked.connect(self.stop_animation)

        # Таймер — только обновление отображения
        self.timer.timeout.connect(self.update_animation)


    def _init_visualizer(self):
        '''
        Инициализация визуализации (только отображение)
        '''
        # Сетка и оси
        self.grid = ShiftedGridItem(size=max(self.space_size), spacing=10, color=(0.5,0.5,0.5,0.7))
        self.grid.add_to_view(self.view)
        self.add_axes()

        # Цель (визуально, не для планирования)
        self.goal_scatter = gl.GLScatterPlotItem(
            pos=np.array([self.goal_pos]),
            size=10,
            color=red,
            pxMode=True
        )
        self.view.addItem(self.goal_scatter)

        # Робот
        self.robot = Robot(
            self.start_pos,
            radius=1.0,
            view=self.view,
            direction=(np.array(self.goal_pos) - np.array(self.start_pos)) / 
							np.linalg.norm(np.array(self.goal_pos) - np.array(self.start_pos)),
            speed=self.speed_robot
        )

        # Препятствия
        self.obstacles = []  # Список препятствий
        for num_obstacle in range(self.num_obstacles):
            pos = np.random.uniform(0, min(self.space_size), 3)
            obstacle = Obstacle(
                num_obstacle,
                pos,
                radius=self.size_obstacles,
                view=self.view,
                space_size=self.space_size,
                speed=self.speed_obstacles
            )
            self.obstacles.append(obstacle)

        # 3D‑радар (визуализация сектора, без вычисления матрицы!)
        self.radar = Radar(
            self.view,
            self.robot.position,
            self.robot.direction,
            self.radar_distance,
            sector_angle=self.sector_angle
        )

        # Линия пути
        self.path_positions = [self.robot.position.copy()]
        self.path_line = gl.GLLinePlotItem(
            pos=np.array(self.path_positions),
            color=QtGui.QColor(255, 255, 0, 255),
            width=1,
            antialias=True
        )
        self.view.addItem(self.path_line)

        # Линия к цели (визуально)
        self.direct_line = gl.GLLinePlotItem(
            pos=np.array([self.robot.position, self.goal_pos]),
            color=QtGui.QColor(255, 255, 255, 127),
            width=0.5,
            antialias=True,
            mode='line_strip'
        )
        self.view.addItem(self.direct_line)


    def add_axes(self):
        axis_length = max(self.space_size)*1.1

        # X
        x_axis = gl.GLLinePlotItem(
            pos=np.array([[0,0,0],[axis_length,0,0]]),
            color=QtGui.QColor(255,0,0),
            width=3,
            antialias=True
        )
        self.view.addItem(x_axis)

        # Y
        y_axis = gl.GLLinePlotItem(
            pos=np.array([[0,0,0],[0,axis_length,0]]),
            color=QtGui.QColor(0,255,0),
            width=3,
            antialias=True
        )
        self.view.addItem(y_axis)

        # Z
        z_axis = gl.GLLinePlotItem(
            pos=np.array([[0,0,0],[0,0,axis_length]]),
            color=QtGui.QColor(0,255,255),
            width=3,
            antialias=True
        )
        self.view.addItem(z_axis)


    def setup_camera(self):
        # Установка камеры
        pos_x = self.space_size[0] * 0.5
        pos_y = self.space_size[1] * 0.5
        pos_z = self.space_size[2] * 0.5
        self.view.setCameraPosition(
            pos=QtGui.QVector3D(pos_x, pos_y, pos_z),
            distance=300,
            elevation=10,
            azimuth=-90
        )


    def update_animation(self):
        """
        Обновление анимации — только отображение.
        Физика, радар‑матрица и планировка вынесены в отдельные процессы.
        """
        # 1) Берём состояние из Process 1 — физика (препятствия и робот)
        while not self.queue_env.empty():
            data = self.queue_env.get()

            # Обновление препятствий
            for ob_data in data["obstacles"]:
                # print("ID:", ob_data["id"], "Pos:", ob_data["pos"])
                obs = self.obstacles[ob_data["id"]]
                obs.set_position(ob_data["pos"])

            # Обновление робота
            robot = data["robot"]

            self.robot.set_position(robot["pos"])
            self.robot.set_direction(robot["direction"])


        # 2) Берём направление и скорость из Process 3 — планировщик
        while not self.queue_planner.empty():

            cmd = self.queue_planner.get()
            self.robot.set_direction(cmd["direction"])
            self.robot.set_speed(cmd["speed"])
            # print("Robot cmd  speed:", cmd["speed"])
            # print("Robot cmd  direction:", cmd["direction"])

        # 3) Движение робота
        # self.robot.move_step()

        # 4) Обновление линий в 3D
        self.path_positions.append(self.robot.get_position().copy())
        self.path_line.setData(pos=np.array(self.path_positions))
        self.direct_line.setData(pos=np.array([self.robot.get_position(), self.goal_pos]))

        # 5) Обновление радар‑окна из Process 2 — радар
        while not self.queue_radar.empty():
            data = self.queue_radar.get()
            self.distance_window.update_matrix(data["matrix"])
            self.distance_window.set_robot_direction(data["angle"], self.startup_test)

        # 6) Обновление 3D‑радара (визуальный сектор)
        self.radar.update(self.robot.position, self.robot.direction)

        # 7) Обновление препятствий (цвет) и проверка столкновений
        self.detected_obstacles = []
    
        for obs in self.obstacles:
            # obs.move()
            dist = np.linalg.norm(obs.get_position() - self.robot.get_position())
            color = (0.0, 1.0, 0.0, 1.0)  # Зелёный
            if self.radar.is_sphere_inside_pyramid(obs.position, obs.radius, self.robot.get_position()):
                color = (1.0, 1.0, 0.0, 1.0)  # Жёлтый
                self.detected_obstacles.append(obs)
            obs.set_color(color)

            # Столкновение
            if check_collision(self.robot, obs):
                obs.set_color((1.0, 0.0, 0.0, 1.0))
                logging.info("Столкновение с препятствием!")
                self.timer.stop()
                return


    def start_animation(self):
        """
        Запуск анимации и процессов (physics, radar, planner).
        MainWindow — только получает результаты.
        """
        if self.timer.isActive():
            return

        #  Показать радар‑окно, если он был закрыт 
        if not self.distance_window.isVisible():
            self.distance_window.show()


        logging.info("Запуск анимации и процессов")
        if self.pause_event_physics:
            self.pause_event_physics.clear()   # снимаем паузу
        if self.pause_event_radar:
            self.pause_event_radar.clear()


        # Параметры для процессов
        # Сформировать robot_data из self.robot
        robot_data = {
            "pos": self.robot.get_position().copy(),
            "direction": self.robot.direction.copy(),
            "speed": self.robot.get_speed(),
        }

        # Сформировать obstacles_data из self.obstacles
        obstacles_data = []
        for obs in self.obstacles:
            obstacles_data.append({
                "id": obs.id_obstacle,
                "pos": obs.get_position().copy(),
                "radius": obs.radius,
                "direction": obs.direction.copy(),
                "speed": obs.speed,
            })


        space_size = self.space_size
        radar_distance = self.radar_distance
        matrix_size = self.matrix_size
        num_sectors = self.matrix_size
        startup_test = self.startup_test

        # 1) Process 1 — физика (движение препятствий и робота)
        if self.p_physics is None:
            self.p_physics = mp.Process(
                target=physics_process,
                kwargs={
                    "queue_env": self.queue_env,
                    "space_size": space_size,
                    "obstacles_data": obstacles_data,
                    "robot_data": robot_data,  
                    "dt": self.fps/1000,
                    "pause_event": self.pause_event_physics,
                }
            )
            self.p_physics.start()

        # 2) Process 2 — радар (матрица дистанций)
        if self.p_radar is None:

            radar_config = {
                "view": self.view,
                "robot_pos": self.robot.position,
                "direction": self.robot.direction,
                "scan_range": self.radar.scan_range,
                "sector_angle": self.sector_angle,           
            }

            self.p_radar = mp.Process(
                target=radar_process,
                kwargs={
                    "queue_env": self.queue_env,
                    "queue_radar": self.queue_radar,
                    "matrix_size": matrix_size,
                    "scan_range": radar_distance,
                    "num_sectors": num_sectors,
                    "startup_test": startup_test,
                    "pause_event": self.pause_event_radar,
                    "radar_config": radar_config,
                }
            )
            self.p_radar.start()

        # 3) Process 3 — планировщик (направление и скорость)
            # направление к цели
        direction_to_goal = np.array(self.goal_pos) - np.array(self.start_pos)
        norm = np.linalg.norm(direction_to_goal)
        if norm > 1e-8:
            start_direction = direction_to_goal / norm
        else:
            start_direction = np.array([1.0, 0.0, 0.0])

        if self.p_planner is None:
            from environment.planner import planner_process
            self.p_planner = mp.Process(
                target=planner_process,
                kwargs={
                    "queue_radar": self.queue_radar,
                    "queue_planner": self.queue_planner,
                    "danger_dist": 5.0,
                    "speed_base": self.speed_robot,
                    "start_direction": start_direction, 
                    "pause_event": self.pause_event_physics,
                }
            )
            self.p_planner.start()

        # 4) Запуск таймера GUI
        self.timer.start(self.fps)  # ~30 FPS


    def pause_animation(self):
        logging.info("Пауза анимации")
        self.timer.stop()
        if self.pause_event_physics:
            self.pause_event_physics.set()   
        if self.pause_event_radar:
            self.pause_event_radar.set()


    def stop_animation(self):
        logging.info("Стоп анимации и процессы")
        self.timer.stop()

        # Опционально можно остановить процессы
        if self.p_physics and self.p_physics.is_alive():
            self.p_physics.terminate()
            self.p_physics.join()
        if self.p_radar and self.p_radar.is_alive():
            self.p_radar.terminate()
            self.p_radar.join()
        if self.p_planner and self.p_planner.is_alive():
            self.p_planner.terminate()
            self.p_planner.join()


    def rotate_vector(self, vec, yaw, pitch):
        """
        Поворот вектора на yaw (горизонтальный) и pitch (вертикальный).
        Чистая математика, не связана с процессами.
        """
        cy, sy = np.cos(yaw), np.sin(yaw)
        cp, sp = np.cos(pitch), np.sin(pitch)
        Rx = np.array([[1, 0, 0],
                       [0, cp, -sp],
                       [0, sp, cp]])
        Ry = np.array([[cy, 0, sy],
                       [0, 1, 0],
                       [-sy, 0, cy]])
        return Rx @ Ry @ vec


    def closeEvent(self, event):
        """GUI закрывается → останавливаем все процессы"""
        logging.info("Закрытие MainWindow, остановка процессов")
        self.timer.stop()

        if self.p_physics and self.p_physics.is_alive():
            self.p_physics.terminate()
            self.p_physics.join()
            logging.info("Physics процесс остановлен")
        if self.p_radar and self.p_radar.is_alive():
            self.p_radar.terminate()
            self.p_radar.join()
            logging.info("Radar процесс остановлен")
        if self.p_planner and self.p_planner.is_alive():
            self.p_planner.terminate()
            self.p_planner.join()
            logging.info("Planner процесс остановлен")

        logging.info("MainWindow закрыт")

        #  Закрываем радар‑окно
        self.distance_window.close()
        logging.info("Радар‑окно закрыто")


        event.accept()  # или event.ignore(), если есть проверки



