
# environment/planner.py
"""
Process 3: планировщик.
Читает матрицу от радара, строит команду и шлёт её в physics.
"""
import time
import logging
import numpy as np


def planner_process(queue_radar, queue_cmd, stop_event, pause_event,
                    goal_pos, robot_pos_ref, danger_dist=8.0,
                    speed_base=1.0, goal_weight=0.7, obstacle_weight=0.3,
                    min_speed_factor=0.1, log_level="INFO"):
    """
    :param robot_pos_ref: multiprocessing.Array или Manager().list
                          — позволяет планировщику знать текущую позицию робота.
                          Простейший вариант — общий np.ndarray через Manager.
    """
    logging.basicConfig(level=log_level,
                        format='[planner] %(asctime)s - %(levelname)s - %(message)s')

    goal = np.asarray(goal_pos, dtype=float)
    speed = float(speed_base)
    current_direction = np.array([1.0, 0.0, 0.0])
    logging.info("planner_process запущен, цель=%s", goal)

    while not stop_event.is_set():
        if pause_event.is_set():
            time.sleep(0.01)
            continue

        # --- Читаем последнюю матрицу ---
        data = None
        while not queue_radar.empty():
            try:
                data = queue_radar.get_nowait()
            except Exception:
                break

        if data is not None:
            matrix = data["matrix"]
            min_dist = data["min_dist"]

            # --- Направление к цели ---
            robot_pos = np.asarray(robot_pos_ref[:], dtype=float)
            to_goal = goal - robot_pos
            d_goal = np.linalg.norm(to_goal)
            if d_goal > 1e-6:
                dir_goal = to_goal / d_goal
            else:
                dir_goal = current_direction

            # --- Направление избегания (из матрицы) ---
            # Берём центральный ряд (elevation=0) — куда идти по азимуту
            n = matrix.shape[0]
            center = n // 2
            forward_row = matrix[center, :]

            # Ищем самый «свободный» азимут
            # Чем БОЛЬШЕ дистанция в столбце — тем безопаснее
            best_col = int(np.argmax(forward_row))
            best_dist = float(forward_row[best_col])

            # Смещение по азимуту в радианах: столбец → угол
            half_angle = np.pi / 4  # половина sector_angle (π/2)
            offset = (best_col - center) / center * half_angle  # [-half, +half]

            # dir_avoid: повёрнутый dir_goal на offset вокруг оси Z
            ca, sa = np.cos(offset), np.sin(offset)
            dir_avoid = np.array([
                ca * dir_goal[0] - sa * dir_goal[1],
                sa * dir_goal[0] + ca * dir_goal[1],
                dir_goal[2],
            ])

            # --- Смешиваем ---
            if min_dist < danger_dist:
                # Опасность: больше веса избеганию
                w_g = goal_weight * (min_dist / danger_dist)
                w_o = 1.0 - w_g
                new_dir = w_g * dir_goal + w_o * dir_avoid
                speed = speed_base * max(min_speed_factor, min_dist / danger_dist)
            else:
                new_dir = dir_goal
                speed = speed_base

            n2 = np.linalg.norm(new_dir)
            if n2 > 1e-8:
                current_direction = new_dir / n2

            # Отправляем команду
            queue_cmd.put({
                "direction": current_direction.copy(),
                "speed": speed,
            })

        time.sleep(0.02)  # 50 Гц

    logging.info("planner_process завершён")





# import time
# import numpy as np

# def planner_process(queue_radar, queue_planner, danger_dist=5.0, speed_base=0.8, start_direction=np.array([1.0, 0.0, 0.0]), pause_event=None):
#     """
#     Process 3: планировщик
#     """
#     best_direction =  start_direction.copy()
#     speed = speed_base
#     matrix_size = 91


#     while True:
#         if pause_event and pause_event.is_set():
   
#             time.sleep(0.01)
#             continue
#         while not queue_radar.empty():
#             data = queue_radar.get()
#             matrix = data["matrix"]
#             angle_rad = data["angle"]
#             min_dist = data["min_dist"]
#             center = matrix_size // 2
#             forward = matrix[center, :]

#             if min_dist < danger_dist:
#                 safe_mask = forward >= danger_dist
#                 if safe_mask.any():
#                     best_beam = np.argmax(forward[safe_mask])
#                     angle_offset = (best_beam - center) * 0.5
#                     angle_rad = angle_rad + np.deg2rad(angle_offset)
#                     best_direction = np.array([np.cos(angle_rad), np.sin(angle_rad), 0.0])
#                     speed = speed_base
#                 else:
#                     speed = 0.01
#             else:
#                 speed = speed_base

#         queue_planner.put({
#             "direction": best_direction.copy(),
#             "speed": speed,
#         })

#         time.sleep(0.001)