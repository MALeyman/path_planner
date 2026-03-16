'''
Планировщик перемещения автономного мобильного объекта в трёхмерной недетерминированной среде.

Автор: Лейман М.А.   
Дата создания: 10.09.2025  

'''



import os

# import signal
# signal.signal(signal.SIGINT, signal.SIG_IGN) 


# # ★★★ ЖЁСТКИЙ FIX для NVIDIA ★★★
# os.environ['PYQTGRAPH_USE_OPENGL'] = '0'
# os.environ["QT_OPENGL"] = "software"
# os.environ["QSG_RENDERER_DEBUG"] = "renderer"  # Debug Qt
# os.environ["QT_QUICK_BACKEND"] = "software"


# ★★★ ЖЁСТКОЕ отключение OpenGL ★★★

# os.environ["QT_OPENGL"] = "software"
# os.environ["QT_OPENGL_IMPLEMENTATION"] = "software"
# os.environ["QSG_RENDERER_DEBUG"] = "renderer"

os.environ["OMP_NUM_THREADS"] = "1"
os.environ["MKL_NUM_THREADS"] = "1"  
os.environ["OPENBLAS_NUM_THREADS"] = "1"
os.environ["PYQTGRAPH_USE_OPENGL"] = "0"


import sys

from PyQt5 import QtWidgets, QtCore, QtGui
import pyqtgraph.opengl as gl
import logging

from objects import Robot, Obstacle
from calculate.check_collision import check_collision
from animate.animate import MainWindow
import time



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

	import numpy as np
	# Параметры
	size = 200
	space_size=(size, size, size)                  			# Размер пространства
	start_pos = [0, 0, space_size[2]/2]                  	# Стартовая позиция робота
	goal_pos = [size*0.8, size*0.8, space_size[2]/2]      	# Целевая точка робота
	# goal_pos = [0, 200, space_size[2]/2] 
	speed_robot = 0.3                           			# Скорость робота
	num_obstacles = 150                         				# Количество препятствий
	size_obstacles = 4                          			# Размеры препятствий
	radar_distance = 50                         			# Дальность обнаружения препятствий (дальность радара)
	sector_angle = np.pi/2                      			# Угол действия радара
	speed_obstacles = 0.2 									# Скорость препятствий
	matrix_size=91
	startup_test = True										# Флаг запуска в тесте



	app = QtWidgets.QApplication(sys.argv)
	window = MainWindow(
	    space_size=space_size,
	    num_obstacles=num_obstacles,
	    start_pos=start_pos,
	    goal_pos=goal_pos,
	    speed_robot=speed_robot,
	    size_obstacles=size_obstacles,
	    radar_distance=radar_distance,
	    sector_angle=sector_angle,
		speed_obstacles=speed_obstacles,
		matrix_size=matrix_size,
		startup_test=startup_test

	)
	
	window.resize(1400, 900)
	window.move(50, 50)
	window.show()
	# ★★★ УЛУЧШЕННАЯ задержка ★★★
	QtCore.QTimer.singleShot(500, lambda: (  # 2 секунды
		window.activateWindow(), 
		window.raise_()
	))
	# QtCore.QTimer.singleShot(500, window.start_animation)
	# window.activateWindow()
	# window.raise_()
	
	sys.exit(app.exec_())





