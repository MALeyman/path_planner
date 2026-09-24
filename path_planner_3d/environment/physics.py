# environment/physics.py

import numpy as np
import time
import multiprocessing as mp

class PhysicsEnvironment:
    def __init__(self, space_size, obstacles_data, robot_data, dt=0.033):
        self.space_size = np.array(space_size, dtype=float)
        self.dt = dt
        self.obstacles = obstacles_data
        self.robot = robot_data

    def step(self):
        # движение препятствий
        for ob in self.obstacles:
            new_pos = ob["pos"] + ob["direction"] * ob["speed"] * self.dt
            for i in range(3):
                if new_pos[i] < 0 or new_pos[i] > self.space_size[i]:
                    ob["direction"][i] = -ob["direction"][i]
                    new_pos[i] = np.clip(new_pos[i], 0, self.space_size[i])
            ob["pos"] = new_pos

        #  движение робота
        self.robot["pos"] = self.robot["pos"] + self.robot["direction"] * self.robot["speed"] * self.dt


        # состояние мира на выход
        return {
            "obstacles": [
                {"id": ob["id"], "pos": ob["pos"].copy(), "radius": ob["radius"]}
                for ob in self.obstacles
            ],
            "robot": {
                "pos": self.robot["pos"].copy(),
                "direction": self.robot["direction"].copy(),
                "speed": self.robot["speed"],
            },
        }


def physics_process(queue_env, space_size, obstacles_data, robot_data, dt=0.033, pause_event=None):
    env = PhysicsEnvironment(
        space_size=space_size,
        obstacles_data=obstacles_data,
        robot_data=robot_data,
        dt=dt,
    )
    while True:
        if pause_event and pause_event.is_set():
  
            time.sleep(dt)
            continue

        state = env.step()
        queue_env.put(state)
        time.sleep(dt)