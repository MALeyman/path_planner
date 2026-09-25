import numpy as np
import pyqtgraph.opengl as gl
from pyqtgraph.opengl import GLMeshItem, MeshData


class Radar:
    def __init__(self, view, robot_pos, direction, scan_range=20, sector_angle=np.pi/4):
        self.view = view
        self.robot_pos = np.array(robot_pos, dtype=float)
        self.direction = np.array(direction, dtype=float)
        self.scan_range = float(scan_range)
        self.sector_angle = float(sector_angle)

        # --- 4 ребра пирамиды ---
        self.edges = []
        for _ in range(4):
            edge = gl.GLLinePlotItem(
                color=(1.0, 1.0, 0.0, 0.9),
                width=1,
                antialias=True,
                glOptions='translucent',
            )
            self.view.addItem(edge)
            self.edges.append(edge)

        # --- Сферическая поверхность ---
        self.sphere_surface = GLMeshItem(
            meshdata=MeshData(),
            smooth=True,
            color=(1.0, 1.0, 0.0, 0.2),    # ★ жёлтая, полупрозрачная
            drawFaces=True,
            drawEdges=False,
            glOptions='translucent',
        )
        self.view.addItem(self.sphere_surface)

        self.update()

    # ------------------------------------------------------------------
    def update(self, robot_pos=None, direction=None):
        if robot_pos is not None:
            self.robot_pos = np.array(robot_pos, dtype=float)
        if direction is not None:
            d = np.array(direction, dtype=float)
            n = np.linalg.norm(d)
            if n > 1e-8:
                self.direction = d / n

        # ★ Пирамида: рёбра идут к углам квадрата на сфере
        # радиус сферы = scan_range, углы пирамиды задаются sector_angle
        base_vertices = self._pyramid_corners_on_sphere()

        for i, edge in enumerate(self.edges):
            next_i = (i + 1) % 4
            edge_points = np.array([
                self.robot_pos,
                base_vertices[i],
                base_vertices[next_i],
            ])
            edge.setData(pos=edge_points, mode='line_strip')

        # ★ Поверхность — часть сферы радиуса scan_range
        md = self.create_spherical_surface()
        self.sphere_surface.setMeshData(
            meshdata=md,
            color=(1.0, 1.0, 0.0, 0.2),    # ★ не перезаписываем цианом
        )

    # ------------------------------------------------------------------
    def _basis(self):
        """
        Ортонормированный базис:
        - forward — направление робота;
        - right   — ЛЕЖИТ В ГОРИЗОНТАЛЬНОЙ ПЛОСКОСТИ (forward × мировая Z);
        - up      — перпендикулярен forward и right.
        """
        forward = self.direction / (np.linalg.norm(self.direction) + 1e-12)

        world_up = np.array([0.0, 0.0, 1.0])
        right = np.cross(forward, world_up)
        rn = np.linalg.norm(right)

        if rn < 1e-6:
            right = np.array([1.0, 0.0, 0.0])
            right = right - forward * np.dot(right, forward)
            right /= np.linalg.norm(right) + 1e-12
        else:
            right = right / rn

        up = np.cross(right, forward)
        return right, up, forward

    # ------------------------------------------------------------------
    def _pyramid_corners_on_sphere(self):
        """
        4 угла пирамиды, лежащие НА сфере радиуса scan_range.
        Пирамида: apex = robot_pos, ось = forward, полуугол = sector_angle/2.
        """
        right, up, forward = self._basis()

        half = self.sector_angle / 2.0
        c = np.cos(half)
        s = np.sin(half)

        # 4 направления на углы квадрата
        dirs = [
            forward * c + right * s + up * s,   # +r, +u
            forward * c - right * s + up * s,   # -r, +u
            forward * c - right * s - up * s,   # -r, -u
            forward * c + right * s - up * s,   # +r, -u
        ]
        # Нормируем и умножаем на scan_range → точки на сфере
        return np.array([
            self.robot_pos + self.scan_range * (d / np.linalg.norm(d))
            for d in dirs
        ])

    # ------------------------------------------------------------------
    def is_sphere_inside_pyramid(self, obs_pos, obs_radius, robot_pos):
        """Сфера внутри пирамиды?"""
        obs_pos = np.array(obs_pos, dtype=float)
        robot_pos = np.array(robot_pos, dtype=float)

        dist = np.linalg.norm(obs_pos - robot_pos)
        if dist > self.scan_range + obs_radius:
            return False
        if dist < 1e-8:
            return True

        # Угловая проверка: направление на препятствие
        direction = self.direction / (np.linalg.norm(self.direction) + 1e-12)
        v = (obs_pos - robot_pos) / dist
        cos_angle = np.clip(np.dot(v, direction), -1.0, 1.0)
        angle = np.arccos(cos_angle)

        # Угловая проверка «квадратного» конуса:
        # для пирамиды — ограничение по двум осям
        right, up, forward = self._basis()
        x = np.dot(v, right)   # отклонение вправо
        y = np.dot(v, up)      # отклонение вверх
        z = np.dot(v, forward) # вдоль оси

        half = self.sector_angle / 2.0
        # Внутри квадратной пирамиды, если |x| ≤ z*tan(half) и |y| ≤ z*tan(half)
        if z <= 0:
            return False
        tan_half = np.tan(half)
        angular_radius = np.arctan2(obs_radius, dist)
        # Запас на радиус сферы
        slack = np.tan(half + angular_radius)

        return abs(x) <= z * slack and abs(y) <= z * slack

    # ------------------------------------------------------------------
    # def create_spherical_surface(self):
    #     """
    #     Сферический сегмент радиуса scan_range, обрезанный квадратной пирамидой.
    #     Единая сетка (phi, theta) с маской по квадрату.
    #     """
    #     phi_steps = 24
    #     theta_steps = 72

    #     half = self.sector_angle / 2.0
    #     tan_half = np.tan(half)

    #     # Угол на диагонали квадрата — предел сферы
    #     phi_max = np.arctan(np.sqrt(2.0) * tan_half)

    #     phi = np.linspace(0, phi_max, phi_steps)
    #     theta = np.linspace(0, 2 * np.pi, theta_steps)
    #     PHI, THETA = np.meshgrid(phi, theta)   # shape (theta_steps, phi_steps)

    #     # Точки на сфере радиуса scan_range
    #     x = self.scan_range * np.sin(PHI) * np.cos(THETA)
    #     y = self.scan_range * np.sin(PHI) * np.sin(THETA)
    #     z = self.scan_range * np.cos(PHI)

    #     # ★ Маска: оставляем точки внутри квадратной пирамиды
    #     #   |x| ≤ z*tan(half)  и  |y| ≤ z*tan(half)
    #     inside = (np.abs(x) <= z * tan_half) & (np.abs(y) <= z * tan_half)

    #     # Точки за пределами пирамиды "сжимаем" к границе
    #     # (проекция на плоскость грани пирамиды)
    #     x_bound = np.clip(x, -z * tan_half, z * tan_half)
    #     y_bound = np.clip(y, -z * tan_half, z * tan_half)

    #     x = np.where(inside, x, x_bound)
    #     y = np.where(inside, y, y_bound)

    #     verts_local = np.column_stack([x.ravel(), y.ravel(), z.ravel()])
    #     verts = self.rotate_points(verts_local, self.direction)
    #     verts += self.robot_pos

    #     md = MeshData()
    #     md.setVertexes(verts.reshape(theta_steps, phi_steps, 3).reshape(-1, 3))

    #     # Faces — обычная регулярная сетка
    #     faces = []
    #     for i in range(theta_steps - 1):
    #         for j in range(phi_steps - 1):
    #             v1 = i * phi_steps + j
    #             v2 = i * phi_steps + (j + 1)
    #             v3 = (i + 1) * phi_steps + j
    #             v4 = (i + 1) * phi_steps + (j + 1)
    #             faces.append([v1, v2, v3])
    #             faces.append([v2, v4, v3])

    #     md.setFaces(np.array(faces, dtype=np.uint32))
    #     return md

    def create_spherical_surface(self):
        """
        Сферический сегмент радиуса scan_range, обрезанный квадратной пирамидой.
        Без сжатия точек — только валидные треугольники.
        """
        phi_steps = 20
        theta_steps = 60

        half = self.sector_angle / 2.0
        tan_half = np.tan(half)
        phi_max = np.arctan(np.sqrt(2.0) * tan_half)

        phi = np.linspace(0, phi_max, phi_steps)
        theta = np.linspace(0, 2 * np.pi, theta_steps)
        PHI, THETA = np.meshgrid(phi, theta)

        x = self.scan_range * np.sin(PHI) * np.cos(THETA)
        y = self.scan_range * np.sin(PHI) * np.sin(THETA)
        z = self.scan_range * np.cos(PHI)

        # ★ Маска — без изменения координат
        inside = (np.abs(x) <= z * tan_half + 1e-9) & (np.abs(y) <= z * tan_half + 1e-9)

        verts_local = np.column_stack([x.ravel(), y.ravel(), z.ravel()])
        verts = self.rotate_points(verts_local, self.direction) + self.robot_pos

        md = MeshData()
        md.setVertexes(verts.reshape(theta_steps, phi_steps, 3).reshape(-1, 3))

        faces = []
        for i in range(theta_steps - 1):
            for j in range(phi_steps - 1):
                # ★ Face только если все 4 вершины внутри
                if not (inside[i, j] and inside[i, j+1]
                        and inside[i+1, j] and inside[i+1, j+1]):
                    continue
                v1 = i * phi_steps + j
                v2 = i * phi_steps + (j + 1)
                v3 = (i + 1) * phi_steps + j
                v4 = (i + 1) * phi_steps + (j + 1)
                faces.append([v1, v2, v3])
                faces.append([v2, v4, v3])

        md.setFaces(np.array(faces, dtype=np.uint32))
        return md


    # ------------------------------------------------------------------
    def rotate_points(self, points, direction):
        rot_matrix = self.direction_to_matrix(direction)
        return np.dot(points, rot_matrix.T)

    def direction_to_matrix(self, direction):
        direction = np.array(direction, dtype=float)
        norm = np.linalg.norm(direction)
        if norm == 0:
            direction = np.array([1, 0, 0], dtype=float)
        else:
            direction = direction / norm

        up = np.array([0, 0, 1], dtype=float)
        if abs(np.dot(up, direction)) > 0.99:
            up = np.array([0, 1, 0], dtype=float)

        x = np.cross(up, direction)
        x_norm = np.linalg.norm(x)
        if x_norm == 0:
            x = np.array([1, 0, 0], dtype=float)
        else:
            x = x / x_norm

        y = np.cross(direction, x)
        return np.column_stack([x, y, direction])
