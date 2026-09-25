# radar/geometry.py
"""
Чистая математика для конуса радара.
Никаких зависимостей от Qt/OpenGL — можно использовать в любом процессе.
"""
import numpy as np


def is_sphere_inside_cone(obs_pos, obs_radius, robot_pos, direction,
                          scan_range, half_angle) -> bool:
    """
    Проверка: сфера (obs_pos, obs_radius) попадает в конус радара.

    :param obs_pos:     центр сферы-препятствия
    :param obs_radius:  радиус сферы
    :param robot_pos:   вершина конуса (робот)
    :param direction:   ось конуса (единичный вектор)
    :param scan_range:  длина конуса
    :param half_angle:  половина угла раствора конуса (радианы)
    """
    obs_pos = np.asarray(obs_pos, dtype=float)
    robot_pos = np.asarray(robot_pos, dtype=float)
    direction = np.asarray(direction, dtype=float)

    v = obs_pos - robot_pos
    dist = np.linalg.norm(v)
    if dist > scan_range + obs_radius:
        return False
    if dist < 1e-8:
        return True  # препятствие на роботе

    # Угол между осью конуса и направлением на препятствие
    d_norm = np.linalg.norm(direction)
    if d_norm < 1e-8:
        return False
    cos_angle = np.dot(v, direction) / (dist * d_norm)
    cos_angle = np.clip(cos_angle, -1.0, 1.0)
    angle = np.arccos(cos_angle)

    # Угловой радиус сферы, видимой из вершины конуса
    angular_radius = np.arctan2(obs_radius, dist)

    return bool(angle <= half_angle + angular_radius)