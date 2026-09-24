import time
import numpy as np

def planner_process(queue_radar, queue_planner, danger_dist=5.0, speed_base=0.8, start_direction=np.array([1.0, 0.0, 0.0]), pause_event=None):
    """
    Process 3: планировщик
    """
    best_direction =  start_direction.copy()
    speed = speed_base
    matrix_size = 91


    while True:
        if pause_event and pause_event.is_set():
   
            time.sleep(0.01)
            continue
        while not queue_radar.empty():
            data = queue_radar.get()
            matrix = data["matrix"]
            angle_rad = data["angle"]
            min_dist = data["min_dist"]
            center = matrix_size // 2
            forward = matrix[center, :]

            if min_dist < danger_dist:
                safe_mask = forward >= danger_dist
                if safe_mask.any():
                    best_beam = np.argmax(forward[safe_mask])
                    angle_offset = (best_beam - center) * 0.5
                    angle_rad = angle_rad + np.deg2rad(angle_offset)
                    best_direction = np.array([np.cos(angle_rad), np.sin(angle_rad), 0.0])
                    speed = speed_base
                else:
                    speed = 0.01
            else:
                speed = speed_base

        queue_planner.put({
            "direction": best_direction.copy(),
            "speed": speed,
        })

        time.sleep(0.001)