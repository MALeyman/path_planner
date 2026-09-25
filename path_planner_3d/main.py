# main.py
"""
Планировщик перемещения автономного мобильного объекта
в трёхмерной недетерминированной среде.

Автор: Лейман М.А.
"""

import warnings
warnings.filterwarnings(
    "ignore",
    message="invalid value encountered in divide",
    category=RuntimeWarning,
    module="pyqtgraph.opengl.MeshData",
)

import os

# ★★★ ФИКС WAYLAND — ДО импорта PyQt ★★★
os.environ.setdefault("QT_QPA_PLATFORM", "xcb")
os.environ.setdefault("PYOPENGL_PLATFORM", "glx")

# Ограничения BLAS-потоков
os.environ.setdefault("OMP_NUM_THREADS", "1")
os.environ.setdefault("MKL_NUM_THREADS", "1")
os.environ.setdefault("OPENBLAS_NUM_THREADS", "1")

import sys
import logging
from pathlib import Path

# ---------- Логирование ----------
PROJECT_ROOT = Path(__file__).parent
LOGS_DIR = PROJECT_ROOT / "logs"
LOGS_DIR.mkdir(exist_ok=True)

from config import config

logging.basicConfig(
    level=getattr(logging, config.log_level, logging.INFO),
    format='%(asctime)s - %(levelname)s - %(message)s',
    handlers=[
        logging.FileHandler(LOGS_DIR / "app.log", mode='a', encoding='utf-8'),
        logging.StreamHandler(),
    ],
)

# ---------- GUI ----------
from PyQt5 import QtWidgets
from animate import MainWindow


def main():
    # Многопроцессность: spawn для совместимости с Qt
    import multiprocessing as mp
    try:
        mp.set_start_method("spawn")
    except RuntimeError:
        pass

    logging.info("Запуск приложения")
    logging.info("QT_QPA_PLATFORM=%s", os.environ.get("QT_QPA_PLATFORM"))
    logging.info("PYOPENGL_PLATFORM=%s", os.environ.get("PYOPENGL_PLATFORM"))

    app = QtWidgets.QApplication(sys.argv)
    window = MainWindow()
    window.show()
    sys.exit(app.exec_())


if __name__ == "__main__":
    main()




