# environment/physics.py
"""
Process 1: физика мира.
Двигает препятствия и робота. Команды роботу приходят из planner через
queue_cmd.
"""
import time
import logging
import numpy as np


class PhysicsEnvironment:
    def __init__(self, space_size, obstacles_data, robot_data, dt=0.033):
        self.space_size = np.array(space_size, dtype=float)
        self.dt = float(dt)

        self.obstacles = []
        for ob in obstacles_data:
            self.obstacles.append({
                "id": ob["id"],
                "pos": np.array(ob["pos"], dtype=float),
                "radius": float(ob["radius"]),
                "direction": np.array(ob["direction"], dtype=float),
                "speed": float(ob["speed"]),
            })

        self.robot = {
            "pos": np.array(robot_data["pos"], dtype=float),
            "direction": np.array(robot_data["direction"], dtype=float),
            "speed": float(robot_data["speed"]),
        }

    # ------------------------------------------------------------------
    def apply_robot_command(self, cmd):
        """Применить команду планировщика."""
        if "direction" in cmd:
            d = np.array(cmd["direction"], dtype=float)
            n = np.linalg.norm(d)
            if n > 1e-8:
                self.robot["direction"] = d / n
        if "speed" in cmd:
            self.robot["speed"] = float(cmd["speed"])

    # ------------------------------------------------------------------
    def step(self):
        # --- Препятствия ---
        for ob in self.obstacles:
            new_pos = ob["pos"] + ob["direction"] * ob["speed"] * self.dt
            for i in range(3):
                if new_pos[i] < ob["radius"]:
                    new_pos[i] = ob["radius"]
                    ob["direction"][i] = -ob["direction"][i]
                elif new_pos[i] > self.space_size[i] - ob["radius"]:
                    new_pos[i] = self.space_size[i] - ob["radius"]
                    ob["direction"][i] = -ob["direction"][i]
            ob["pos"] = new_pos

        # --- Робот ---
        self.robot["pos"] = self.robot["pos"] + self.robot["direction"] * self.robot["speed"] * self.dt
        # Границы
        for i in range(3):
            self.robot["pos"][i] = np.clip(
                self.robot["pos"][i], 0.0, self.space_size[i]
            )

        # --- Состояние для внешнего мира ---
        return {
            "obstacles": [
                {
                    "id": ob["id"],
                    "pos": ob["pos"].copy(),
                    "radius": ob["radius"],
                    "direction": ob["direction"].copy(),
                    "speed": ob["speed"],
                }
                for ob in self.obstacles
            ],
            "robot": {
                "pos": self.robot["pos"].copy(),
                "direction": self.robot["direction"].copy(),
                "speed": self.robot["speed"],
            },
        }


# ---------------------------------------------------------------------------
def physics_process(queue_env_radar, queue_env_gui, queue_cmd,
                    stop_event, pause_event,
                    space_size, obstacles_data, robot_data,
                    dt=0.033, log_level="INFO"):
    """
    Process 1.
    Пишет состояние в ДВЕ очереди (radar и GUI), читает команды из queue_cmd.
    """
    logging.basicConfig(level=log_level,
                        format='[physics] %(asctime)s - %(levelname)s - %(message)s')

    env = PhysicsEnvironment(space_size, obstacles_data, robot_data, dt=dt)
    logging.info("physics_process запущен")

    while not stop_event.is_set():
        if pause_event.is_set():
            time.sleep(0.01)
            continue

        # Применяем все накопившиеся команды (важна последняя)
        last_cmd = None
        while not queue_cmd.empty():
            try:
                last_cmd = queue_cmd.get_nowait()
            except Exception:
                break
        if last_cmd is not None:
            env.apply_robot_command(last_cmd)

        state = env.step()

        # Дублируем состояние в две очереди
        queue_env_radar.put(state)
        queue_env_gui.put(state)

        time.sleep(dt)

    logging.info("physics_process завершён")








# import numpy as np
# import time
# import multiprocessing as mp

# class PhysicsEnvironment:
#     def __init__(self, space_size, obstacles_data, robot_data, dt=0.033):
#         self.space_size = np.array(space_size, dtype=float)
#         self.dt = dt
#         self.obstacles = obstacles_data
#         self.robot = robot_data

#     def step(self):
#         # движение препятствий
#         for ob in self.obstacles:
#             new_pos = ob["pos"] + ob["direction"] * ob["speed"] * self.dt
#             for i in range(3):
#                 if new_pos[i] < 0 or new_pos[i] > self.space_size[i]:
#                     ob["direction"][i] = -ob["direction"][i]
#                     new_pos[i] = np.clip(new_pos[i], 0, self.space_size[i])
#             ob["pos"] = new_pos

#         #  движение робота
#         self.robot["pos"] = self.robot["pos"] + self.robot["direction"] * self.robot["speed"] * self.dt


#         # состояние мира на выход
#         return {
#             "obstacles": [
#                 {"id": ob["id"], "pos": ob["pos"].copy(), "radius": ob["radius"]}
#                 for ob in self.obstacles
#             ],
#             "robot": {
#                 "pos": self.robot["pos"].copy(),
#                 "direction": self.robot["direction"].copy(),
#                 "speed": self.robot["speed"],
#             },
#         }


# def physics_process(queue_env, space_size, obstacles_data, robot_data, dt=0.033, pause_event=None):
#     env = PhysicsEnvironment(
#         space_size=space_size,
#         obstacles_data=obstacles_data,
#         robot_data=robot_data,
#         dt=dt,
#     )
#     while True:
#         if pause_event and pause_event.is_set():
  
#             time.sleep(dt)
#             continue

#         state = env.step()
#         queue_env.put(state)
#         time.sleep(dt)