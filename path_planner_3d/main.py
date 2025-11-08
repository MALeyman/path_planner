'''
Планировщик перемещения автономного мобильного объекта в трёхмерной недетерминированной среде.

Автор: Лейман М.А.   
Дата создания: 10.09.2025  

'''



import os
import sys
import numpy as np
from PyQt5 import QtWidgets, QtCore, QtGui
import pyqtgraph.opengl as gl
import logging

from objects import Robot, Obstacle
from calculate.check_collision import check_collision
from animate.animate import MainWindow






# Путь к папке logs на уровне проекта
project_root = os.path.dirname(os.path.abspath(__file__))  # Папка текущего файла
logs_dir = os.path.join(project_root, 'logs')

# Создаём папку, если её нет
os.makedirs(logs_dir, exist_ok=True)

log_path = os.path.join(logs_dir, 'app.log')


# Настройка логгера — вывод логов в файл и/или консоль
logging.basicConfig(
    level=logging.INFO,  # уровень логирования
    format='%(asctime)s - %(levelname)s - %(message)s',  # формат сообщения
    handlers=[
        logging.FileHandler(log_path, mode='a', encoding='utf-8'),  # лог в файл
        logging.StreamHandler()  # лог в консоль
    ]
)

# logging.debug("Отладочное сообщение")
# logging.info("Информационное сообщение")
# logging.warning("Предупреждение")
# logging.error("Ошибка")
# logging.critical("Критическая ошибка")




if __name__ == "__main__":


    # Параметры
    space_size=(100, 100, 100)                  # Размер пространства
    start_pos = [0, 0, space_size[2]/2]         # Стартовая позиция робота
    goal_pos = [100, 100, space_size[2]/2]      # Целевая точка робота
    speed_robot = 3.0                           # Скорость робота
    num_obstacles = 100                         # Количество препятствий
    size_obstacles = 4                          # Размеры препятствий
    radar_distance = 20                         # Дальность обнаружения препятствий (дальность радара)


    app = QtWidgets.QApplication(sys.argv)
    window = MainWindow(
        space_size=space_size,
        num_obstacles=num_obstacles,
        start_pos=start_pos,
        goal_pos=goal_pos,
        speed_robot=speed_robot,
        size_obstacles=size_obstacles,
        radar_distance=radar_distance
    )

    window.show()
    window.showNormal()
    window.activateWindow()
    window.raise_()

    sys.exit(app.exec_())


