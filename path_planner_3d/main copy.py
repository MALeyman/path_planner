
def generate_3d_control(u_opt, orientation, current_pos, target_pos, params):
    # Расчёт управляющих сигналов
    omega_psi = params.k_p_psi * (math.atan2(u_opt[1], u_opt[0]) - orientation.yaw)
    omega_theta = params.k_p_theta * (math.asin(u_opt[2]) - orientation.pitch)
    v_z = params.k_z * (target_pos.z - current_pos.z)

    # Ограничения
    omega_psi = clamp(omega_psi, -params.omega_max, params.omega_max)
    omega_theta = clamp(omega_theta, -params.omega_max_pitch, params.omega_max_pitch)
    v_z = clamp(v_z, -params.v_z_max, params.v_z_max)

    return {'omega_psi': omega_psi, 'omega_theta': omega_theta, 'v_z': v_z}

def evaluate_cluster(cluster, u_goal, sigma, sigma_slope):
    # Отклонение от цели
    delta = math.acos(np.dot(cluster.center_dir, u_goal))
    # Штраф за уклон
    slope_penalty = math.exp(-cluster.avg_slope**2 / (2 * sigma_slope**2))
    # Приоритет
    priority = cluster.volume * math.exp(-delta**2 / (2 * sigma**2)) * slope_penalty
    return priority


def calculate_3d_goal(current, target):
    dx = target.x - current.x
    dy = target.y - current.y
    dz = target.z - current.z
    theta = math.atan2(dy, dx)
    phi = math.asin(dz / math.sqrt(dx**2 + dy**2 + dz**2))
    return theta, phi



def dv_h_3d_planner(current_pos, target_pos, orientation, radar_3d, params, global_map=None):
    # Шаг 1: расчёт направления на цель в 3D
    theta_goal, phi_goal = calculate_3d_goal(current_pos, target_pos)
    u_goal = spherical_to_cartesian(theta_goal, phi_goal)

    # Шаг 2: построение 3D DVH
    H_3d = build_3d_dvh(radar_3d, params.phi_h, params.phi_v,
                       params.n_sectors_h, params.n_sectors_v)

    # Шаг 3: фильтрация и учёт динамики
    H_filtered = apply_threshold(H_3d, params.d_safe)
    H_dynamic = apply_derivative_weight(H_filtered, params.k_h, params.k_v)

    # Шаг 4: поиск безопасных кластеров
    clusters = find_safe_clusters(H_dynamic, params.d_safe)

    # Шаг 5: оценка кластеров с учётом рельефа
    for cluster in clusters:
        cluster.priority = evaluate_cluster(cluster, u_goal, params.sigma, params.sigma_slope)

    # Шаг 6: интеграция с глобальной картой
    if global_map:
        clusters = integrate_with_global_map(clusters, global_map, params.lambda_vis)

    # Шаг 7: выбор оптимального кластера
    best_cluster = select_best_cluster(clusters)
    if not best_cluster:
        return handle_stuck_case(H_dynamic, params)  # режим поиска выхода

    # Шаг 8: 3D-свёртка для габаритов
    H_convolved = apply_3d_convolution(H_dynamic, current_speed, params.robot_radius)

    # Шаг 9: формирование управления
    u_opt = best_cluster.direction
    control_signals = generate_3d_control(u_opt, orientation, current_pos, target_pos, params)

    # Шаг 10: проверка ограничений траектории
    trajectory = build_trajectory(current_pos, u_opt, params.d_lookahead)
    if not is_trajectory_safe(trajectory, H_convolved):
        control_signals = reduce_speed(control_signals, params)

    return control_signals, trajectory



