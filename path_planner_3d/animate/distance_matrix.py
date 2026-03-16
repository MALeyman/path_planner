''' 
   Классы:
     DistanceMatrixWindow:  Визуализации матрицы дистанций
     MatrixDistance:  заполнение матрицы дистанций
'''
# animate/distance_matrix.py
import pyqtgraph as pg
import numpy as np
import math
from pyqtgraph.Qt import QtWidgets
import logging

class DistanceMatrixWindow(QtWidgets.QWidget):
    """ОТОБРАЖЕНИЕ матрицы"""
    def __init__(self, matrix_size=91, radar_distance=30):
        super().__init__()
        self.setWindowTitle("Матрица дистанций 91×91")
        self.resize(500, 500)
        self.radar_distance = radar_distance

        layout = QtWidgets.QVBoxLayout(self)
        self.plot = pg.PlotWidget(title="Радар (DVH)")
        self.plot.setAspectLocked(True)
        self.img = pg.ImageItem()
        self.plot.addItem(self.img)
        layout.addWidget(self.plot)
        
        # Серый градиент
        lut = np.zeros((256, 4), dtype=np.ubyte)
        for i in range(256):
            gray = int(255 * i / 255.0)
            lut[i] = [gray, gray, gray, 255]
        self.img.setLookupTable(lut)
        
        self.matrix_size = matrix_size
        
        self.matrix = np.full((matrix_size, matrix_size), self.radar_distance)
        self.center_point = None
        self.direction_arrow = None
        self._add_center_marker()
        self.update_matrix(self.matrix)
    
    def update_matrix(self, matrix_distance):
        self.matrix = np.clip(matrix_distance, 0, self.radar_distance)
        normalized = (self.matrix / self.radar_distance * 255).astype(np.uint8)
        self.img.setImage(normalized)
    
    def _add_center_marker(self):
        """✅ ФИКСИРОВАННАЯ КРАСНАЯ ТОЧКА в центре [45.5,45.5]"""
        cx, cy = self.matrix_size/2, self.matrix_size/2
        # Красная точка (центр робота)
        self.center_point = pg.ScatterPlotItem(
            x=[cx], y=[cy], 
            pen=pg.mkPen(None), 
            brush=pg.mkBrush('r', size=10)  # Красный круг 10px
        )
        self.plot.addItem(self.center_point)
    
    def set_robot_direction(self, angle_rad, startup_test):
        """Стрелка направления (ОСТАВЛЯЕТСЯ)"""
        # Удаляем старую стрелку
        if self.direction_arrow is not None:
            self.plot.removeItem(self.direction_arrow)
        
        cx, cy = self.matrix_size/2, self.matrix_size/2
        length = 12  # Длиннее!
        ex = cx + length * np.cos(angle_rad)
        ey = cy + length * np.sin(angle_rad)
        
        self.direction_arrow = pg.PlotDataItem(
            [cx, ex], [cy, ey], 
            pen=pg.mkPen('lime', width=4)  # Зелёная стрелка!
        )
        self.plot.addItem(self.direction_arrow)


class MatrixDistance:
    """Матрица дистанций"""
    def __init__(self, matrix_size=91, scan_range=20.0, num_sectors=91):
        self.matrix_size = matrix_size
        self.scan_range = scan_range
        self.num_sectors = num_sectors
        self.matrix_distance = np.full((matrix_size, matrix_size), scan_range)
    
    def matrix_distance_calculation(self, direction_move, obstacles_info, current_pos, current_speed, startup_test):
        """ВСЕ препятствия на радаре!"""
        self.matrix_distance.fill(self.scan_range)  # ✅ Белый фон
        
        min_dist_obstacle = self.scan_range
        detected_count = 0
        
        print("🔍 Обнаружено препятствий:", len(obstacles_info))
        robot_pos = np.array(current_pos)
        for i, obstacle_data in enumerate(obstacles_info):
            # obs_dist = obstacle_data['distance']
            # obs_dir = np.array(obstacle_data['direction'])
            
            obs_pos = obstacle_data.get_position()
            obs_dist = np.linalg.norm(obs_pos - robot_pos)
            obs_dir = obs_pos - robot_pos
            print("robot_pos === ", robot_pos)
            print("obs_pos === ", obs_pos)
            
            azimuth, elevation = self._angles_between_directions(direction_move, obs_dir)
            print("Направление на препятствие obs_dir - ", obs_dir)
            print("azimuth", azimuth)
            print("elevation", elevation)
            
            angular_size = self._angular_size(obstacle_data.radius, obs_dist)

            # ✅ ВСЕГДА рисуем квадрат!
            self._fill_matrix_sector(obs_dist, azimuth, elevation, angular_size)
            detected_count += 1
            
            print(f"  ID:{i} dist:{obs_dist:.1f}м azim:{azimuth:.1f}° size:{angular_size:.1f}°")
            
            # Только проверка угрозы
            if abs(azimuth) < angular_size:
                min_dist_obstacle = min(min_dist_obstacle, obs_dist)
                print(f"   🔥 ПО КУРСУ!")
        
        print(f"📊 Показано: {detected_count}/{len(obstacles_info)}")
        return self.matrix_distance, min_dist_obstacle, np.ones(len(obstacles_info))



    # def matrix_distance_calculation(self, direction_move, detected_obstacles, current_pos, current_speed, startup_test):
    #     """Заполнение матрицы дистанций"""
    #     robot_pos = np.array(current_pos)
    #     min_dist_obstacle = self.scan_range
    #     detected_obs = np.zeros(len(detected_obstacles))
        
    #     # print("Обнаружены препятствия:")
        
    #     # Очищаем матрицу
    #     self.matrix_distance.fill(self.scan_range)
        
    #     if detected_obstacles:
    #         logging.warning("Обнаружены препятствия")
    #     for i, obstacle_data in enumerate(detected_obstacles):

    #         obs_pos = obstacle_data.get_position()
    #         obs_dist = np.linalg.norm(obs_pos - robot_pos)
    #         obs_dir = obs_pos - robot_pos
 
    #         detected_obs[i] = 1
    #         print(f"  ID:{obstacle_data.id_obstacle} {obs_dist:5.1f}м")
            
    #         # Углы между направлением и препятствием
    #         azimuth_angle, elevation_angle = self._angles_between_directions(direction_move, obs_dir)

    #         # Угловой размер препятствия
    #         angular_size = self._angular_size(obstacle_data.radius, obs_dist)
    #         if startup_test:
    #             logging.info("azimuth_angle: %.1f°", azimuth_angle)
    #             logging.info("elevation_angle: %.1f°", elevation_angle)
    #             logging.info("УГЛОВОЙ размер:: %.1f°", angular_size)
          
    #         # Препятствие по курсу?
    #         if abs(azimuth_angle) < angular_size and abs(elevation_angle) < angular_size:
    #             logging.warning("Препятствия по курсу!")
    #             min_dist_obstacle = min(min_dist_obstacle, obs_dist)
            
    #         # Заполняем матрицу
    #         self._fill_matrix_sector(obs_dist, azimuth_angle, elevation_angle, angular_size, startup_test=True)
        
    #     return self.matrix_distance, min_dist_obstacle, detected_obs
    

    def _fill_matrix_sector(self, obs_dist, azimuth_angle, elevation_angle=0, angular_size=0, startup_test=True):
        angular_size = max(1, math.ceil(angular_size))
        
        # ✅ АЗИМУТ -180°..+180° → 0..90 (0°=45, -180°=0, +180°=90)
        center_x = int(45.0 + (azimuth_angle))  # -30° → 45-7.5=37.5
        
        # ✅ ELEVATION -45°..+45° → 0..90 (0°=45)
        center_y = int(45.0 + (elevation_angle))  # -20° → 45-10=35
        
        center_x = np.clip(center_x, 0, self.matrix_size - 1)
        center_y = np.clip(center_y, 0, self.matrix_size - 1)
        
        half_size = angular_size // 2
        logging.info("azimuth_angle: %.1f°", azimuth_angle)
        logging.info("elevation_angle: %.1f°", elevation_angle)
        logging.info("УГЛОВОЙ размер:: %.1f°", angular_size)
          
        logging.info("🎯 azim=%.1f→X:%d elev=%.1f→Y:%d", 
                    azimuth_angle, center_x, elevation_angle, center_y)
        
        # ✅ PyQtGraph: X=elevation, Y=azimuth
        for dx in range(-half_size, half_size + 1):
            for dy in range(-half_size, half_size + 1):
                px = np.clip(center_y + dy, 0, self.matrix_size - 1)  # Y=azimuth
                py = np.clip(center_x + dx, 0, self.matrix_size - 1)  # X=elevation
                self.matrix_distance[py, px] = min(self.matrix_distance[py, px], obs_dist)



   
    def _angles_between_directions(self, dir1, dir2):
        """✅ Угол dir2 ОТНОСИТЕЛЬНО dir1 (курса)"""
        dir1 = dir1 / np.linalg.norm(dir1)
        dir2 = dir2 / np.linalg.norm(dir2)
        print("курс робота === ", dir1)
        print("Направление на препятствие === ", dir2)

        # ✅ Углы абсолютные от оси X
        angle1 = np.arctan2(dir1[1], dir1[0])  # 45° = 0.785 рад
        angle2 = np.arctan2(dir2[1], dir2[0])  # dir2 угол
        
        # ✅ ОТНОСИТЕЛЬНЫЙ азимут!
        azimuth = np.degrees(angle1 - angle2)
        print("Угол 1 == ", angle1)
        print("Угол 2  == ", angle2)
        print("Азимут == ", azimuth)

        # Нормализация -180°..+180°
        azimuth = ((azimuth + 180) % 360) - 180
        print("Азимут 2 == ", azimuth)
        elevation = np.degrees(np.arcsin(dir2[2]))
        print("Возвышение  == ", elevation)

        return azimuth, elevation


    def _angular_size(self, radius, distance):
        """ 
           Угловой размер препятствия
        """
        if distance < 0.1:
            return 45.0
        return math.degrees(math.asin(radius / distance))
    

    def update_distance_matrix(self, robot, detected_obstacles, startup_test):
        """ 
        Обновление  МАТРИЦЫ ДИСТАНЦИЙ
        """
        robot_pos = robot.get_position()
        direction_move = robot.direction
        robot_speed = robot.speed

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














