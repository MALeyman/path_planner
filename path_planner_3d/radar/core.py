# radar/core.py
"""
MatrixDistance: вычисление матрицы дистанций.
radar_process: Process 2 — читает queue_env, пишет queue_radar.
"""
import time
import logging
import numpy as np

from .geometry import is_sphere_inside_cone
from config import config

# radar/core.py

class MatrixDistance:
    def __init__(self, matrix_size=91, scan_range=20.0, sector_angle=np.pi / 2):
        self.matrix_size = matrix_size
        self.scan_range = float(scan_range)
        self.sector_angle = float(sector_angle)
        self.half_angle = self.sector_angle / 2.0
        self.matrix_distance = np.full(
            (matrix_size, matrix_size), scan_range, dtype=float
        )

    # ------------------------------------------------------------------
    def _basis_from(self, direction):
        """Единый базис (right, up, forward) — используется и для углов, и для логов."""
        d = direction / (np.linalg.norm(direction) + 1e-12)
        world_up = np.array([0.0, 0.0, 1.0])
        if abs(np.dot(d, world_up)) > 0.99:
            world_up = np.array([0.0, 1.0, 0.0])
        right = np.cross(d, world_up)
        right /= np.linalg.norm(right) + 1e-12
        up = np.cross(right, d)
        return right, up, d

    def _angles(self, direction, v):
        """Углы в радианах: azimuth (влево/вправо) и elevation (вверх/вниз)."""
        right, up, d = self._basis_from(direction)
        v_norm = v / (np.linalg.norm(v) + 1e-12)
        x = np.dot(v_norm, right)
        y = np.dot(v_norm, up)
        z = np.dot(v_norm, d)
        azimuth = np.arctan2(x, z)
        elevation = np.arctan2(y, z)
        return azimuth, elevation

    @staticmethod
    def _angular_size(radius, dist):
        if dist < 1e-8:
            return np.pi
        return 2.0 * np.arctan2(radius, dist)

    def _fill_sector(self, dist, azimuth, elevation, angular_size):
        """
        Заполняет квадратную область в матрице.
        Возвращает (row, col) — центр области.
        """
        n = self.matrix_size
        center = (n - 1) / 2.0
        scale = (n - 1) / self.sector_angle

        cx = int(round(center + azimuth * scale))
        cy = int(round(center - elevation * scale))

        half_px = max(1, int(round(angular_size * scale / 2.0)))

        x0 = max(0, cx - half_px)
        x1 = min(n, cx + half_px + 1)
        y0 = max(0, cy - half_px)
        y1 = min(n, cy + half_px + 1)

        if x0 < x1 and y0 < y1:
            sub = self.matrix_distance[y0:y1, x0:x1]
            np.minimum(sub, dist, out=sub)

        return int(np.clip(cy, 0, n - 1)), int(np.clip(cx, 0, n - 1))

    def update(self, robot, detected_obstacles):
        """
        :return: (matrix, min_dist, infos)
            infos — список dict:
              id, world, local, dist, azimuth, elevation, matrix_row, matrix_col
        """
        self.matrix_distance.fill(self.scan_range)
        min_dist = self.scan_range

        robot_pos = np.asarray(robot["pos"], dtype=float)
        direction = np.asarray(robot["direction"], dtype=float)
        right, up, forward = self._basis_from(direction)

        infos = []

        for obs in detected_obstacles:
            obs_pos = np.asarray(obs["pos"], dtype=float)
            obs_radius = float(obs["radius"])

            v = obs_pos - robot_pos
            dist = float(np.linalg.norm(v))
            if dist < 1e-8:
                continue

            azimuth, elevation = self._angles(direction, v)
            angular_size = self._angular_size(obs_radius, dist)
            row, col = self._fill_sector(dist, azimuth, elevation, angular_size)

            # Локальные координаты: (вправо, вверх, вперёд) в метрах
            v_norm = v / dist
            x_local = float(np.dot(v_norm, right)) * dist
            y_local = float(np.dot(v_norm, up)) * dist
            z_local = float(np.dot(v_norm, forward)) * dist

            if abs(azimuth) <= self.half_angle + angular_size / 2:
                if dist < min_dist:
                    min_dist = dist

            infos.append({
                "id": obs.get("id", -1),
                "world": (float(obs_pos[0]), float(obs_pos[1]), float(obs_pos[2])),
                "local": (x_local, y_local, z_local),
                "dist": dist,
                "azimuth": float(np.degrees(azimuth)),
                "elevation": float(np.degrees(elevation)),
                "matrix_row": row,
                "matrix_col": col,
            })

        return self.matrix_distance, min_dist, infos

def radar_process(queue_env, queue_radar, stop_event, pause_event,
                  matrix_size, scan_range, sector_angle, log_level="INFO"):
    """
    Process 2: читает состояние мира, строит матрицу дистанций.
    НЕ создаёт Qt/OpenGL объектов.
    """
    logging.basicConfig(
        level=log_level,
        format='[radar] %(asctime)s - %(levelname)s - %(message)s'
    )

    matrix_calc = MatrixDistance(
        matrix_size=matrix_size,
        scan_range=scan_range,
        sector_angle=sector_angle,
    )
    half_angle = sector_angle / 2.0

    # ★ Инициализация параметров логирования — ДО цикла
    LOG_INTERVAL = 1.0        # секунд; поставьте 0.0, чтобы логировать каждый кадр
    last_log_time = 0.0

    logging.info("radar_process запущен")

    while not stop_event.is_set():
        if pause_event.is_set():
            time.sleep(0.01)
            continue

        try:
            state = queue_env.get(timeout=0.1)
        except Exception:
            continue

        obstacles = state["obstacles"]
        robot = state["robot"]

        robot_pos = np.asarray(robot["pos"], dtype=float)
        direction = np.asarray(robot["direction"], dtype=float)

        # Отбор препятствий внутри конуса
        detected = []
        for obs in obstacles:
            if is_sphere_inside_cone(
                obs_pos=obs["pos"],
                obs_radius=obs["radius"],
                robot_pos=robot_pos,
                direction=direction,
                scan_range=scan_range,
                half_angle=half_angle,
            ):
                detected.append(obs)

        # ★ update возвращает 3 значения — matrix, min_dist, infos
        matrix, min_dist, infos = matrix_calc.update(robot, detected)

        angle_world = float(np.arctan2(direction[1], direction[0]))

        # Очищаем очередь перед записью
        while not queue_radar.empty():
            try:
                queue_radar.get_nowait()
            except Exception:
                break

        queue_radar.put({
            "matrix": matrix.copy(),
            "angle": angle_world,
            "min_dist": min_dist,
            "detected_count": len(detected),
        })

        # ── ЛОГ ──
        if infos:
            now = time.time()
            if now - last_log_time >= LOG_INTERVAL:
                logging.info(
                    "Обнаружено %d/%d  робот world=(%.1f, %.1f, %.1f)  "
                    "min_dist=%.2f",
                    len(infos), len(obstacles),
                    robot_pos[0], robot_pos[1], robot_pos[2],
                    min_dist,
                )
                for info in infos:
                    wx, wy, wz = info["world"]
                    lx, ly, lz = info["local"]
                    logging.info(
                        "  id=%3d  "
                        "world=(%7.1f,%7.1f,%7.1f)  "
                        "local=(%7.2f,%7.2f,%7.2f)  "
                        "dist=%6.2f  azim=%6.1f°  elev=%6.1f°  "
                        "matrix[row=%2d, col=%2d]",
                        info["id"],
                        wx, wy, wz,
                        lx, ly, lz,
                        info["dist"],
                        info["azimuth"],
                        info["elevation"],
                        info["matrix_row"], info["matrix_col"],
                    )
                last_log_time = now

    logging.info("radar_process завершён")