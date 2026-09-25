# animate/distance_matrix.py
"""Окно визуализации матрицы дистанций (2D)."""
import numpy as np
import pyqtgraph as pg
from pyqtgraph.Qt import QtWidgets


class DistanceMatrixWindow(QtWidgets.QWidget):
    def __init__(self, matrix_size=91, radar_distance=30.0):
        super().__init__()
        self.setWindowTitle(f"Матрица дистанций {matrix_size}×{matrix_size}")
        self.resize(500, 500)

        self.radar_distance = float(radar_distance)
        self.matrix_size = int(matrix_size)

        layout = QtWidgets.QVBoxLayout(self)
        self.plot = pg.PlotWidget(title="Радар (DVH)")
        self.plot.setAspectLocked(True)
        self.img = pg.ImageItem()
        self.plot.addItem(self.img)
        layout.addWidget(self.plot)
        self.plot.setLabel('bottom', 'Azimuth (лев ← 0 → прав)')
        self.plot.setLabel('left', 'Elevation (верх ← 0 → низ)')


        self.plot.invertY(True)
        # Серый градиент: 0 = чёрный, 255 = белый
        lut = np.zeros((256, 4), dtype=np.ubyte)
        for i in range(256):
            lut[i] = [i, i, i, 255]
        self.img.setLookupTable(lut)

        # Красная точка в центре
        c = self.matrix_size / 2.0
        c = (self.matrix_size - 1) / 2.0 
        self.center_point = pg.ScatterPlotItem(
            x=[c], y=[c],
            pen=pg.mkPen(None),
            brush=pg.mkBrush('r'),
            size=10,
        )
        self.plot.addItem(self.center_point)

        self.direction_arrow = None
        self.matrix = np.full((matrix_size, matrix_size), radar_distance, dtype=float)
        self.update_matrix(self.matrix)

    # ------------------------------------------------------------------
    def update_matrix(self, matrix):
        self.matrix = np.asarray(matrix, dtype=float)
        norm = self.matrix / self.radar_distance
        norm = np.nan_to_num(norm, nan=1.0)
        norm = np.clip(norm, 0.0, 1.0)
        norm = norm ** 0.35
        self.img.setImage((norm * 255).astype(np.uint8), axisOrder='row-major')

    # ------------------------------------------------------------------
    def set_robot_direction(self, angle_rad, startup_test=True):
        if self.direction_arrow is not None:
            self.plot.removeItem(self.direction_arrow)

        c = self.matrix_size / 2.0
        c = (self.matrix_size - 1) / 2.0 
        length = 12.0
        ex = c + length * np.cos(angle_rad)
        ey = c + length * np.sin(angle_rad)

        self.direction_arrow = pg.PlotDataItem(
            [c, ex], [c, ey],
            pen=pg.mkPen('lime', width=4),
        )
        self.plot.addItem(self.direction_arrow)

