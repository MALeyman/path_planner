# animate/main_window.py
"""
GUI-процесс.
Только отображение. Ничего не вычисляет.
"""
import time
import logging
import multiprocessing as mp

import numpy as np
from pyqtgraph.Qt import QtCore, QtWidgets, QtGui
import pyqtgraph.opengl as gl

from config import config
from objects import Robot, Obstacle
from calculate.check_collision import check_collision
from radar import Radar
from animate.distance_matrix import DistanceMatrixWindow
from environment import physics_process, planner_process
from radar import radar_process


# ---------------------------------------------------------------------------
class ShiftedGridItem:
    """Сетка на полу сцены."""

    def __init__(self, size=10, spacing=10, color=(0.5, 0.5, 0.5, 1)):
        self.size = size
        self.spacing = spacing
        self.color = color
        self.lines = []

    def add_to_view(self, view):
        for i in np.arange(0, self.size + self.spacing, self.spacing):
            pts_x = np.array([[0, i, 0], [self.size, i, 0]])
            line_x = gl.GLLinePlotItem(pos=pts_x, color=self.color, width=1)
            view.addItem(line_x)
            self.lines.append(line_x)

            pts_y = np.array([[i, 0, 0], [i, self.size, 0]])
            line_y = gl.GLLinePlotItem(pos=pts_y, color=self.color, width=1)
            view.addItem(line_y)
            self.lines.append(line_y)


# ---------------------------------------------------------------------------
class MainWindow(QtWidgets.QWidget):

    def __init__(self):
        super().__init__()
        self.setWindowTitle("3D Планировщик")

        self._init_state()
        self._create_widgets()
        self._setup_layout()
        self._setup_signals()
        self._init_visualizer()
        self._setup_processes()

    # ------------------------------------------------------------------
    def _init_state(self):
        self.cfg = config

        # multiprocessing
        self.queue_env_radar = mp.Queue(maxsize=2)  # physics → radar
        self.queue_env_gui = mp.Queue(maxsize=2)    # physics → GUI
        self.queue_radar = mp.Queue(maxsize=2)      # radar   → GUI
        self.queue_cmd = mp.Queue(maxsize=4)        # planner → physics

        self.stop_event = mp.Event()
        self.pause_event_physics = mp.Event()
        self.pause_event_radar = mp.Event()

        # Manager для разделяемой позиции робота (планировщик → physics)
        self.manager = mp.Manager()
        self.shared_robot_pos = self.manager.list(
            [float(x) for x in self.cfg.space.start_pos]
        )

        self.p_physics = None
        self.p_radar = None
        self.p_planner = None
        self.processes_started = False

        # Таймер отрисовки
        self.timer = QtCore.QTimer()

    # ------------------------------------------------------------------
    def _create_widgets(self):
        self.view = gl.GLViewWidget()
        self.view.renderOrder = 'frontToBack'

        self.distance_window = DistanceMatrixWindow(
            matrix_size=self.cfg.radar.matrix_size,
            radar_distance=self.cfg.radar.distance,
        )
        self.distance_window.show()

        self.start_button = QtWidgets.QPushButton("Старт")
        self.pause_button = QtWidgets.QPushButton("Пауза")
        self.stop_button = QtWidgets.QPushButton("Стоп")

    # ------------------------------------------------------------------
    def _setup_layout(self):
        main_layout = QtWidgets.QHBoxLayout()
        left_layout = QtWidgets.QVBoxLayout()
        left_layout.addWidget(self.view, stretch=1)

        btn_layout = QtWidgets.QHBoxLayout()
        btn_layout.addWidget(self.start_button)
        btn_layout.addWidget(self.pause_button)
        btn_layout.addWidget(self.stop_button)
        left_layout.addLayout(btn_layout)
        main_layout.addLayout(left_layout, stretch=3)

        outer = QtWidgets.QVBoxLayout()
        outer.addLayout(main_layout)
        self.setLayout(outer)

        w, h = self.cfg.gui.window_size
        px, py = self.cfg.gui.window_pos
        self.resize(w, h)
        self.move(px, py)
        self.setMinimumSize(*self.cfg.gui.min_size)

    # ------------------------------------------------------------------
    def _setup_signals(self):
        self.start_button.clicked.connect(self.start_animation)
        self.pause_button.clicked.connect(self.pause_animation)
        self.stop_button.clicked.connect(self.stop_animation)
        self.timer.timeout.connect(self.update_animation)

    # ------------------------------------------------------------------
    def _init_visualizer(self):
        # --- Сетка и оси ---
        max_size = max(self.cfg.space.size)
        grid = ShiftedGridItem(
            size=max_size,
            spacing=self.cfg.gui.grid_spacing,
            color=(0.5, 0.5, 0.5, 0.7),
        )
        grid.add_to_view(self.view)
        self._add_axes(max_size * 1.1)

        # --- Цель ---
        goal = np.array(self.cfg.space.goal_pos, dtype=float)
        self.goal_scatter = gl.GLScatterPlotItem(
            pos=np.array([goal]),
            size=10,
            color=(1, 0, 0, 1),
            pxMode=True,
        )
        self.view.addItem(self.goal_scatter)

        # --- Робот ---
        start = np.array(self.cfg.space.start_pos, dtype=float)
        direction = goal - start
        self.robot = Robot(
            start,
            radius=self.cfg.robot.radius,
            view=self.view,
            direction=direction,
            speed=self.cfg.robot.speed,
        )

        # --- Препятствия ---
        if self.cfg.obstacles.seed is not None:
            np.random.seed(self.cfg.obstacles.seed)
        self.obstacles = []
        for i in range(self.cfg.obstacles.count):
            pos = np.random.uniform(0, min(self.cfg.space.size), 3)
            obs = Obstacle(
                id_obstacle=i,
                position=pos,
                radius=self.cfg.obstacles.radius,
                view=self.view,
                space_size=self.cfg.space.size,
                speed=self.cfg.obstacles.speed,
            )
            self.obstacles.append(obs)

        # --- Радар (визуальный) ---
        self.radar = Radar(
            self.view,
            self.robot.position,
            self.robot.direction,
            scan_range=self.cfg.radar.distance,
            sector_angle=self.cfg.radar.sector_angle,
        )

        # --- Линия пути ---
        self.path_positions = [self.robot.position.copy()]
        self.path_line = gl.GLLinePlotItem(
            pos=np.array(self.path_positions),
            color=(1, 1, 0, 1),
            width=1,
            antialias=True,
        )
        self.view.addItem(self.path_line)

        # --- Линия к цели ---
        self.direct_line = gl.GLLinePlotItem(
            pos=np.array([self.robot.position, goal]),
            color=(1, 1, 1, 0.5),
            width=0.5,
            antialias=True,
            mode='line_strip',
        )
        self.view.addItem(self.direct_line)

        # --- Камера ---
        sx, sy, sz = self.cfg.space.size
        self.view.setCameraPosition(
            pos=QtGui.QVector3D(sx * 0.5, sy * 0.5, sz * 0.5),
            distance=self.cfg.gui.camera_distance,
            elevation=self.cfg.gui.camera_elevation,
            azimuth=self.cfg.gui.camera_azimuth,
        )

    def _add_axes(self, length):
        axes = [
            ([[0, 0, 0], [length, 0, 0]], (1.0, 0.0, 0.0, 1.0)),   # X — красный
            ([[0, 0, 0], [0, length, 0]], (0.0, 1.0, 0.0, 1.0)),   # Y — зелёный
            ([[0, 0, 0], [0, 0, length]], (0.0, 1.0, 1.0, 1.0)),   # Z — циан
        ]
        for pts, color in axes:
            item = gl.GLLinePlotItem(
                pos=np.array(pts, dtype=float),
                color=color,
                width=3,
                antialias=True,
            )
            self.view.addItem(item)

    # ------------------------------------------------------------------
    def _setup_processes(self):
        """Создаёт процессы, но НЕ запускает их."""
        pass  # запуск в start_animation

    # ------------------------------------------------------------------
    def start_animation(self):
        if self.timer.isActive():
            return
        if not self.distance_window.isVisible():
            self.distance_window.show()

        logging.info("Запуск анимации")

        self.stop_event.clear()
        self.pause_event_physics.clear()
        self.pause_event_radar.clear()

        if not self.processes_started:
            self._launch_processes()
            self.processes_started = True

        self.timer.start(int(1000 / self.cfg.gui.fps))

    def _launch_processes(self):
        # --- Данные для процессов ---
        robot_data = {
            "pos": self.robot.get_position(),
            "direction": self.robot.get_direction(),
            "speed": self.robot.get_speed(),
        }
        obstacles_data = [obs.to_dict() for obs in self.obstacles]

        dt = 1.0 / self.cfg.gui.fps

        # --- Process 1: physics ---
        self.p_physics = mp.Process(
            target=physics_process,
            kwargs=dict(
                queue_env_radar=self.queue_env_radar,
                queue_env_gui=self.queue_env_gui,
                queue_cmd=self.queue_cmd,
                stop_event=self.stop_event,
                pause_event=self.pause_event_physics,
                space_size=self.cfg.space.size,
                obstacles_data=obstacles_data,
                robot_data=robot_data,
                dt=dt,
                log_level=self.cfg.log_level,
            ),
            daemon=True,
        )
        self.p_physics.start()

        # --- Process 2: radar ---
        self.p_radar = mp.Process(
            target=radar_process,
            kwargs=dict(
                queue_env=self.queue_env_radar,
                queue_radar=self.queue_radar,
                stop_event=self.stop_event,
                pause_event=self.pause_event_radar,
                matrix_size=self.cfg.radar.matrix_size,
                scan_range=self.cfg.radar.distance,
                sector_angle=self.cfg.radar.sector_angle,
                log_level=self.cfg.log_level,
            ),
            daemon=True,
        )
        self.p_radar.start()

        # --- Process 3: planner ---
        self.p_planner = mp.Process(
            target=planner_process,
            kwargs=dict(
                queue_radar=self.queue_radar,
                queue_cmd=self.queue_cmd,
                stop_event=self.stop_event,
                pause_event=self.pause_event_physics,
                goal_pos=self.cfg.space.goal_pos,
                robot_pos_ref=self.shared_robot_pos,
                danger_dist=self.cfg.planner.danger_dist,
                speed_base=self.cfg.robot.speed,
                goal_weight=self.cfg.planner.goal_weight,
                obstacle_weight=self.cfg.planner.obstacle_weight,
                min_speed_factor=self.cfg.planner.min_speed_factor,
                log_level=self.cfg.log_level,
            ),
            daemon=True,
        )
        self.p_planner.start()

        logging.info("Все процессы запущены")

    # ------------------------------------------------------------------
    def pause_animation(self):
        if not self.timer.isActive():
            # Снять паузу
            logging.info("Продолжение")
            self.pause_event_physics.clear()
            self.pause_event_radar.clear()
            self.timer.start(int(1000 / self.cfg.gui.fps))
        else:
            logging.info("Пауза")
            self.timer.stop()
            self.pause_event_physics.set()
            self.pause_event_radar.set()

    # ------------------------------------------------------------------
    def stop_animation(self):
        logging.info("Стоп")
        self.timer.stop()
        self.stop_event.set()

        # Обновляем shared позицию, чтобы не мешать физике
        for p in (self.p_physics, self.p_radar, self.p_planner):
            if p is not None and p.is_alive():
                p.join(timeout=2.0)
                if p.is_alive():
                    p.terminate()
                    p.join(timeout=1.0)

        self.p_physics = None
        self.p_radar = None
        self.p_planner = None
        self.processes_started = False

    # ------------------------------------------------------------------
    def update_animation(self):
        # --- Состояние мира от physics ---
        last_state = None
        while not self.queue_env_gui.empty():
            try:
                last_state = self.queue_env_gui.get_nowait()
            except Exception:
                break

        if last_state is not None:
            # Препятствия
            for ob_data in last_state["obstacles"]:
                obs = self.obstacles[ob_data["id"]]
                obs.set_position(ob_data["pos"])

            # Робот
            robot = last_state["robot"]
            self.robot.set_position(robot["pos"])
            self.robot.set_direction(robot["direction"])
            self.robot.set_speed(robot["speed"])

            # Обновляем shared-позицию для планировщика
            self.shared_robot_pos[:] = [float(x) for x in robot["pos"]]

            # Линии
            self.path_positions.append(self.robot.get_position())
            if len(self.path_positions) > 5000:
                self.path_positions.pop(0)
            self.path_line.setData(pos=np.array(self.path_positions))
            self.direct_line.setData(
                pos=np.array([self.robot.get_position(),
                              self.cfg.space.goal_pos])
            )

        # --- Данные радара ---
        last_radar = None
        while not self.queue_radar.empty():
            try:
                last_radar = self.queue_radar.get_nowait()
            except Exception:
                break

        if last_radar is not None:
            self.distance_window.update_matrix(last_radar["matrix"])
            self.distance_window.set_robot_direction(last_radar["angle"])

        # --- Визуальный радар следует за роботом ---
        self.radar.update(self.robot.position, self.robot.direction)

        # --- Подсветка препятствий и проверка столкновений ---
        for obs in self.obstacles:
            if self.radar_contains(obs):
                obs.set_color((1.0, 1.0, 0.0, 1.0))
            else:
                obs.set_color((0.0, 1.0, 0.0, 1.0))

            if check_collision(self.robot, obs):
                obs.set_color((1.0, 0.0, 0.0, 1.0))
                logging.warning("Столкновение с препятствием id=%d", obs.id)
                self.timer.stop()
                self.stop_event.set()
                return

    # ------------------------------------------------------------------
    def radar_contains(self, obs):
        """Локальная проверка попадания в конус — без вызова radar."""
        from radar.geometry import is_sphere_inside_cone
        return is_sphere_inside_cone(
            obs_pos=obs.position,
            obs_radius=obs.radius,
            robot_pos=self.robot.position,
            direction=self.robot.direction,
            scan_range=self.cfg.radar.distance,
            half_angle=self.cfg.radar.sector_angle / 2.0,
        )

    # ------------------------------------------------------------------
    def closeEvent(self, event):
        logging.info("Закрытие MainWindow")
        self.timer.stop()
        self.stop_event.set()

        for p in (self.p_physics, self.p_radar, self.p_planner):
            if p is not None and p.is_alive():
                p.join(timeout=2.0)
                if p.is_alive():
                    p.terminate()
                    p.join(timeout=1.0)

        try:
            self.manager.shutdown()
        except Exception:
            pass

        self.distance_window.close()
        event.accept()