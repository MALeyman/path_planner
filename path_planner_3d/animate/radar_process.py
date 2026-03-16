import numpy as np
import pyqtgraph as pg
import multiprocessing as mp
from queue import Empty
import time
import logging
from dataclasses import dataclass

@dataclass
class RadarData:
	matrix: np.ndarray
	target_pos: tuple
	robot_dir: tuple
	speed: float

class RadarProcessor:
	def __init__(self, num_sectors=64):
		self.num_sectors = num_sectors
		self.data_queue = mp.Queue()
		
	def process(self, robot_pos, robot_dir, obstacles, scan_range):
		"""Расчёт матрицы в отдельном процессе"""
		matrix = np.full((self.num_sectors, self.num_sectors), np.nan)
		
		# Сканирование радара
		for obs in obstacles:
			dist = np.linalg.norm(np.array(obs['pos']) - np.array(robot_pos))
			if dist < scan_range:
				# Углы
				az = np.arctan2(obs['pos'][1]-robot_pos[1], obs['pos'][0]-robot_pos[0])
				el = np.arcsin((obs['pos'][2]-robot_pos[2]) / dist)
				
				i = int((az + np.pi) / (2*np.pi) * self.num_sectors)
				j = int((el + np.pi/2) / np.pi * self.num_sectors)
				
				if 0 <= i < self.num_sectors and 0 <= j < self.num_sectors:
					matrix[i,j] = dist
		
		data = RadarData(matrix, (0,0), robot_dir, 1.0)
		self.data_queue.put(data)
		return matrix
