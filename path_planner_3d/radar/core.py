# radar/core.py
''' 
    Вычисление матрицы дистанций
'''
import numpy as np
import math
import logging
import time
from radar import Radar


class MatrixDistance:
    """Матрица дистанций"""
    def __init__(self, matrix_size=91, scan_range=20.0, num_sectors=91):
        self.matrix_size = matrix_size
        self.scan_range = scan_range
        self.num_sectors = num_sectors
        self.matrix_distance = np.full((matrix_size, matrix_size), scan_range)
    
    
    def matrix_distance_calculation(self, direction_move, obstacles_info, current_pos, current_speed, startup_test):
        """ВСЕ препятствия на радаре!"""
        self.matrix_distance.fill(self.scan_range)  # 
        
        min_dist_obstacle = self.scan_range
        detected_count = 0
        
        print("Обнаружено препятствий:", len(obstacles_info))
        robot_pos = np.array(current_pos)
        for i, obstacle_data in enumerate(obstacles_info):
  
            obs_pos = obstacle_data["pos"]
            obs_dist = np.linalg.norm(obs_pos - robot_pos)
            obs_dir = obs_pos - robot_pos
            # print("robot_pos === ", robot_pos)
            # print("obs_pos === ", obs_pos)
            
            azimuth, elevation = self._angles_between_directions(direction_move, obs_dir)
            # print("Направление на препятствие obs_dir - ", obs_dir)
            # print("azimuth", azimuth)
            # print("elevation", elevation)
            
            angular_size = self._angular_size(obstacle_data["radius"], obs_dist)

            # ВСЕГДА рисуем квадрат!
            self._fill_matrix_sector(obs_dist, azimuth, elevation, angular_size)
            detected_count += 1
            
            print(f"  ID:{i} dist:{obs_dist:.1f}м azim:{azimuth:.1f}° elevation:{elevation:.1f}°  size:{angular_size:.1f}°")
            
            # Только проверка угрозы
            if abs(azimuth) < angular_size:
                min_dist_obstacle = min(min_dist_obstacle, obs_dist)
                print(f"   🔥 ПО КУРСУ!")
        
        print(f"Показано: {detected_count}/{len(obstacles_info)}")
        return self.matrix_distance, min_dist_obstacle, np.ones(len(obstacles_info))


    def _fill_matrix_sector(self, obs_dist, azimuth_angle, elevation_angle=0, angular_size=0, startup_test=True):
        angular_size = max(1, math.ceil(angular_size))
        
        #  АЗИМУТ -180°..+180° → 0..90 (0°=45, -180°=0, +180°=90)
        center_x = int(46.0 + (azimuth_angle))  # -30° → 45-7.5=37.5
        
        #  ELEVATION -45°..+45° → 0..90 (0°=46)
        center_y = int(46.0 + (elevation_angle))  
        
        center_x = np.clip(center_x, 0, self.matrix_size - 1)
        center_y = np.clip(center_y, 0, self.matrix_size - 1)
        
        half_size = angular_size // 2
        logging.info("azimuth_angle: %.1f°", azimuth_angle)
        logging.info("elevation_angle: %.1f°", elevation_angle)
        logging.info("УГЛОВОЙ размер:: %.1f°", angular_size)
        print("Дистанция ", obs_dist)  
        logging.info("azim=%.1f→X:%d elev=%.1f→Y:%d", 
                    azimuth_angle, center_x, elevation_angle, center_y)
        
        # X=elevation, Y=azimuth
        for dx in range(-half_size, half_size + 1):
            for dy in range(-half_size, half_size + 1):
                px = np.clip(center_y + dy, 0, self.matrix_size - 1)  # Y=azimuth
                py = np.clip(center_x + dx, 0, self.matrix_size - 1)  # X=elevation
                self.matrix_distance[py, px] = min(self.matrix_distance[py, px], obs_dist)

   
    def _angles_between_directions(self, dir1, dir2):
        """Угол dir2 ОТНОСИТЕЛЬНО dir1 (курса)"""

        norm1 = np.linalg.norm(dir1)
        norm2 = np.linalg.norm(dir2)
        if norm1 < 1e-8 or norm2 < 1e-8:
            return 0.0, 0.0

        dir1 = dir1 / norm1
        dir2 = dir2 / norm2

        # Углы абсолютные от оси X
        angle1 = np.arctan2(dir1[1], dir1[0])  # 45° = 0.785 рад
        angle2 = np.arctan2(dir2[1], dir2[0])  # dir2 угол
        
        # ОТНОСИТЕЛЬНЫЙ азимут!
        azimuth = np.degrees(angle1 - angle2)

        # Нормализация -180°..+180°
        azimuth = ((azimuth + 180) % 360) - 180
  
        elevation = np.degrees(np.arcsin(dir2[2]))

        return azimuth, elevation


    def _angular_size(self, radius, dist):
        """ 
           Угловой размер препятствия
        """
        if dist < 1e-8:
            return 90.0
        return np.rad2deg(2 * np.arctan2(radius, dist))


    def update_distance_matrix(self, robot, detected_obstacles, startup_test):
        """ 
        Обновление  МАТРИЦЫ ДИСТАНЦИЙ
        """
        robot_pos = robot["pos"]
        direction_move = robot["direction"]
        robot_speed = robot["speed"]

        # detected_obstacles
        matrix, min_dist, detected = self.matrix_distance_calculation(
            direction_move, detected_obstacles, robot_pos, robot_speed, startup_test
        )

        # 3.
        min_val = np.min(matrix)
        idx_flat = np.argmin(matrix)
        i, j = np.unravel_index(idx_flat, matrix.shape)
        print("Минимальный элемент:", min_val)
        print("Индексы (i, j):", i, j)

        # Отображение
        angle = np.arctan2(direction_move[1], direction_move[0])
        return matrix, min_dist, detected, angle




def radar_process(queue_env, queue_radar, matrix_size=91, scan_range=20, num_sectors=91, startup_test=False, pause_event=None, radar_config=None, ):
    """
    Process 2 — радар: строит matrix дистанций из состояния среды.
    """
    matrix_calc = MatrixDistance(
        matrix_size=matrix_size,
        scan_range=scan_range,
        num_sectors = num_sectors,
        
    )

    # 1. Воссоздаём radar в процессе
    if radar_config:
        radar = Radar(
            view=radar_config["view"],
            robot_pos=radar_config["robot_pos"],
            direction=radar_config["direction"],
            scan_range=radar_config["scan_range"],
            sector_angle=radar_config["sector_angle"],
        )


    while True:
        if pause_event and pause_event.is_set():
            time.sleep(0.001)
            continue
        if not queue_env.empty():
            data = queue_env.get()
            obstacles_info = data["obstacles"]
            robot = data["robot"]

            detected_obstacles = []
            robot_pos = np.array(robot["pos"])

            for obs in obstacles_info:
                obs_pos = obs["pos"]
                obs_radius = obs["radius"]

                # Добавим обнаруженные препятствия в список
                if radar.is_sphere_inside_pyramid(obs_pos, obs_radius, robot_pos):
                    detected_obstacles.append({
                        "pos": obs_pos,
                        "radius": obs_radius,
                        "id": obs.get("id", None)
                    })


            matrix, min_dist, detected, angle = matrix_calc.update_distance_matrix(
                robot=robot,
                detected_obstacles=detected_obstacles,
                startup_test=startup_test,
            )

            queue_radar.put({
                "matrix": matrix,
                "angle": angle,
                "min_dist": min_dist,
            })

        time.sleep(0.001)  #

