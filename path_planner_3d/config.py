# config.py
"""
Все настройки проекта в одном месте.
Меняйте здесь, а не в main.py.
"""
from dataclasses import dataclass, field
from typing import Optional, Tuple, List
import numpy as np


@dataclass
class SpaceConfig:
    """Размеры пространства и стартовые позиции."""
    size: Tuple[float, float, float] = (200.0, 200.0, 200.0)
    start_pos: Optional[List[float]] = None  # None -> автоматически
    goal_pos: Optional[List[float]] = None   # None -> автоматически

    def __post_init__(self):
        s = self.size
        if self.start_pos is None:
            self.start_pos = [0.0, 0.0, s[2] / 2]
        if self.goal_pos is None:
            self.goal_pos = [s[0] * 0.8, s[1] * 0.8, s[2] / 2]


@dataclass
class RobotConfig:
    radius: float = 1.0
    speed: float = 1.9


@dataclass
class ObstaclesConfig:
    count: int = 100
    radius: float = 4.0
    speed: float = 1.8
    seed: Optional[int] = None  # для воспроизводимости


@dataclass
class RadarConfig:
    distance: float = 50.0        # дальность
    sector_angle: float = np.pi / 2  # полный угол конуса (не половина!)
    matrix_size: int = 91         # размер матрицы дистанций


@dataclass
class PlannerConfig:
    danger_dist: float = 8.0       # дистанция тревоги
    goal_weight: float = 0.7       # вес стремления к цели
    obstacle_weight: float = 0.3   # вес избегания
    min_speed_factor: float = 0.1  # минимальная скорость при опасности


@dataclass
class GUIConfig:
    fps: int = 30
    window_size: Tuple[int, int] = (1400, 900)
    window_pos: Tuple[int, int] = (50, 50)
    min_size: Tuple[int, int] = (1000, 700)
    camera_distance: float = 300.0
    camera_elevation: float = 10.0
    camera_azimuth: float = -90.0
    grid_spacing: float = 10.0


@dataclass
class Config:
    """Главный конфиг."""
    space: SpaceConfig = field(default_factory=SpaceConfig)
    robot: RobotConfig = field(default_factory=RobotConfig)
    obstacles: ObstaclesConfig = field(default_factory=ObstaclesConfig)
    radar: RadarConfig = field(default_factory=RadarConfig)
    planner: PlannerConfig = field(default_factory=PlannerConfig)
    gui: GUIConfig = field(default_factory=GUIConfig)
    log_level: str = "INFO"


# ★ Единственный экземпляр, который импортируется везде ★
config = Config()

