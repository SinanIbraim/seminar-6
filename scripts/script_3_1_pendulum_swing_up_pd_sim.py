import numpy as np
import matplotlib.pyplot as plt
import matplotlib.cm as cm
import os
import sys
from matplotlib.lines import Line2D

# --- Настройка путей для импорта --- 
SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
PARENT_OF_SCRIPTS_DIR = os.path.dirname(SCRIPT_DIR)
if PARENT_OF_SCRIPTS_DIR not in sys.path:
    sys.path.append(PARENT_OF_SCRIPTS_DIR)

# --- Импорты из src --- 
try:
    # from src.systems.pendulum_motor_system import PendulumMotorSystem # Заменено
    from src.systems.pendulum import Pendulum
    from src.systems.motor_system import MotorSystem
    from src.controllers.energy_swing_up_controller import EnergySwingUpController
    from src.controllers.pi_controller import PIController
    from src.controllers.pd_controller import PDController
except ImportError as e:
    print(f"Ошибка импорта: {e}. Убедитесь, что структура проекта верна и PYTHONPATH настроен.")
    print(f"SCRIPT_DIR: {SCRIPT_DIR}")
    print(f"PARENT_OF_SCRIPTS_DIR (должен содержать src): {PARENT_OF_SCRIPTS_DIR}")
    print(f"sys.path: {sys.path}")
    exit()

# --- Параметры симуляции --- 
dT = 0.005  # Шаг времени (уменьшим для большей точности ручного Эйлера)
T_sim = 6.0  # Общее время симуляции
N_steps = int(T_sim / dT) # Количество шагов

# --- Параметры маятника ---
m_pend = 0.6  # Масса маятника (кг)
l_pend = 0.5  # Длина маятника до центра масс (м)
g_pend = 9.81 # Ускорение свободного падения (м/с^2)
damping_pend = 0.00 # Коэффициент вязкого трения маятника (Н*м*с/рад) - НОВЫЙ ПАРАМЕТР

# --- Параметры мотора ---
motor_gamma = 1/10  # Постоянная времени мотора (с)

# --- Начальное состояние системы ---
# [theta, theta_dot, motor_torque]
initial_state_vector = np.array([0.01, 0.0, 0.0]) # Маятник почти внизу, покоится, момент 0
initial_pendulum_state = initial_state_vector[0:2]
initial_motor_state = np.array([initial_state_vector[2]]) # Должен быть 1D массив для MotorSystem

# --- Параметры контроллера энергетической раскачки ---
u_max_swing_up = 1  # Максимальный ЖЕЛАЕМЫЙ момент от EnergySwingUpController (Нм)
# Этот момент будет целью для ПИ-регулятора мотора.
s2_deadband_swing_up = 0.05 # Порог для угловой скорости в EnergySwingUpController
energy_target_factor = 1.0 # Изменено с 1.02 на 1.0

# --- Параметры ПД-регулятора для стабилизации маятника (взяты из script_2_1) ---
J_pend = m_pend * l_pend**2 # Момент инерции маятника, нужен для расчета Kp, Kd
sqrt_g_l = np.sqrt(g_pend / l_pend)
alpha_pend_stab = 1.5 * sqrt_g_l 
Kp_pd_pend = J_pend * alpha_pend_stab**2 - m_pend * g_pend * l_pend
Kd_pd_pend = 2 * J_pend * alpha_pend_stab
tau_max_pd_output = u_max_swing_up # Ограничим максимальный момент ПД тем же значением, что и у раскачки

# --- Параметры переключения на ПД-контроллер ---
# Переключаемся, когда маятник близок к верхнему положению и скорость мала
angle_threshold_for_pd = np.pi * 0.1  # (rad) Отклонение от np.pi, при котором возможно переключение
velocity_threshold_for_pd = 0.5    # (rad/s) Максимальная скорость для переключения

# --- Параметры ПИ-регулятора для контура момента мотора ---
# Цель: обеспечить быстрое отслеживание tau_desired моментом tau_m
# Используем формулы Kp = 2*alpha*gamma - 1, Ki = alpha^2*gamma
# alpha_motor определяет скорость сходимости контура момента. 
# Чем больше alpha_motor, тем быстрее реакция, но больше риск неустойчивости или шумов.
alpha_motor_desired = 100.0  # Желаемое расположение полюсов (-alpha_motor) для контура момента (рад/с)

Kp_motor = 2 * motor_gamma * alpha_motor_desired - 1
Ki_motor = motor_gamma * (alpha_motor_desired**2)
# Ограничение интеграла для предотвращения wind-up. u_max_swing_up - это целевой момент,
# фактическая команда \'a\' может быть больше, если мотор слабый.
# Пусть команда \'a\' может быть до ~2*u_max_swing_up или чуть больше.
# Интегральный член Ki * integral_error должен быть сопоставим с Kp * error.
# Если error ~ u_max_swing_up, то Kp*error ~ Kp_motor * u_max_swing_up.
# Ki*integral_limit ~ Kp_motor * u_max_swing_up => integral_limit ~ (Kp_motor/Ki_motor) * u_max_swing_up
# integral_limit_motor = (Kp_motor / Ki_motor) * u_max_swing_up if Ki_motor > 0 else u_max_swing_up * 5
# Более простой подход: взять лимит немного больше максимального ожидаемого установившегося управления.
# Если мотор должен выдавать u_max_swing_up, то \'a\' должно быть u_max_swing_up в установившемся режиме.
# Но чтобы быстро достичь этого, \'a\' может быть больше.
integral_limit_motor = u_max_swing_up * 2.0 

print(f"Параметры ПИ-контроллера мотора: Kp_motor = {Kp_motor:.2f}, Ki_motor = {Ki_motor:.2f}, integral_limit = {integral_limit_motor:.2f}")
if Kp_motor <=0 :
    print(f"ПРЕДУПРЕЖДЕНИЕ: Kp_motor ({Kp_motor:.2f}) <= 0. Это может привести к неверной работе ПИ-регулятора мотора.")
    print(f"  alpha_motor_desired ({alpha_motor_desired}) должен быть > 1 / (2 * motor_gamma) = {1/(2*motor_gamma):.2f}")

# --- Инициализация системы и контроллеров ---
# pendulum_sys = PendulumMotorSystem(
#     initial_state=initial_state_vector.copy(),
#     mass=m_pend,
#     length=l_pend,
#     gravity=g_pend,
#     motor_gamma=motor_gamma
# )

pendulum_actual_sys = Pendulum(
    mass=m_pend,
    length=l_pend,
    gravity=g_pend,
    damping=damping_pend, # Используем новый параметр
    initial_state=initial_pendulum_state.copy()
)
# Установка имен и единиц для маятника, так как класс Pendulum не вызывает super().__init__
pendulum_actual_sys.name = "Pendulum"
pendulum_actual_sys.state_names = ['Angle theta', 'Ang. vel. theta_dot']
pendulum_actual_sys.state_units = ['rad', 'rad/s']


motor_actual_sys = MotorSystem(
    initial_state=initial_motor_state.copy(),
    gamma=motor_gamma
)
# MotorSystem уже устанавливает name, state_names, state_units, control_input_names, control_input_units

energy_ctrl = EnergySwingUpController(
    mass=m_pend,
    length=l_pend,
    gravity=g_pend,
    u_max=u_max_swing_up,
    pendulum_state_indices=(0, 1), 
    s2_deadband_threshold=s2_deadband_swing_up,
    energy_target_factor=energy_target_factor
)

pi_motor_ctrl = PIController(
    Kp=Kp_motor,
    Ki=Ki_motor,
    dt=dT,
    output_dim=1, 
    integral_limit=integral_limit_motor,
    initial_integral_error=np.array([0.0])
)

# Инициализация ПД-контроллера для стабилизации
pd_stabilizer = PDController(
    Kp=Kp_pd_pend,
    Kd=Kd_pd_pend,
    dt=dT,
    control_limits=(-tau_max_pd_output, tau_max_pd_output),
    target_is_angle=True, # Важно для корректной обработки ошибки по углу (pi - theta)
    name="Pendulum_PD_Stabilizer"
)

# --- Подготовка для хранения истории ---
time_history = np.zeros(N_steps + 1)
# state_history = np.zeros((N_steps + 1, pendulum_sys.state_dim)) # Было (N+1, 3)
pendulum_state_history = np.zeros((N_steps + 1, 2))
motor_state_history = np.zeros((N_steps + 1, 1))
control_a_history = np.zeros((N_steps, pi_motor_ctrl.output_dim)) # Зависит от pi_motor_ctrl
tau_desired_history = np.zeros((N_steps, energy_ctrl.output_dim)) # Зависит от energy_ctrl
current_energy_history = np.zeros(N_steps + 1)
# Новые массивы для истории
# [d_theta_dt, d_theta_dot_dt, d_motor_torque_dt]
derivatives_history = np.zeros((N_steps, 3))
Ep_history = np.zeros(N_steps + 1) # Потенциальная энергия
Ek_history = np.zeros(N_steps + 1) # Кинетическая энергия
# pi_integral_error_history = np.zeros((N_steps + 1, pi_motor_ctrl.output_dim)) # Удалено
controller_mode_history = np.zeros(N_steps) # 0 для Energy, 1 для PD
t_switch_to_pd = -1.0 # Время переключения на ПД, -1 если не переключились

# --- Начальные значения ---
# current_state = initial_state_vector.copy()
# state_history[0, :] = current_state
current_pendulum_state = initial_pendulum_state.copy()
current_motor_state = initial_motor_state.copy()

pendulum_state_history[0, :] = current_pendulum_state
motor_state_history[0, :] = current_motor_state
time_history[0] = 0.0
# Расчет начальных энергий по формулам из EnergySwingUpController
# V = mgl(1 - cos(theta))
# T = 0.5 * J * theta_dot^2, где J = m*l^2
Ep_history[0] = m_pend * g_pend * l_pend * (1 - np.cos(current_pendulum_state[0]))
Ek_history[0] = 0.5 * J_pend * current_pendulum_state[1]**2
current_energy_history[0] = Ep_history[0] + Ek_history[0]
# pi_integral_error_history[0, :] = pi_motor_ctrl.get_integral_error().flatten() # Удалено


print(f"Запуск ручной симуляции: {N_steps} шагов, dT={dT}c, T_sim={T_sim}c")
# --- Основной цикл симуляции (ручной) ---
active_controller_is_pd = False # Флаг, какой контроллер активен

for i in range(N_steps):
    t_current = i * dT
    
    current_theta = current_pendulum_state[0]
    current_theta_dot = current_pendulum_state[1]

    # Логика переключения контроллеров
    if not active_controller_is_pd: # Если еще работает EnergySwingUp
        # Проверяем условия для переключения на ПД
        # Угол должен быть близок к PI (верхнее положение)
        # Скорость должна быть достаточно мала
        
        # Новое вычисление is_near_top с нормализацией угла:
        error_angle_to_pi = current_theta - np.pi
        # Нормализуем эту ошибку в диапазон [-pi, pi]
        normalized_angle_error = (error_angle_to_pi + np.pi) % (2 * np.pi) - np.pi
        # Теперь проверяем, достаточно ли мала абсолютная величина этой нормализованной ошибки
        is_near_top = abs(normalized_angle_error) < angle_threshold_for_pd
        
        is_slow_enough = abs(current_theta_dot) < velocity_threshold_for_pd
        
        if is_near_top and is_slow_enough:
            active_controller_is_pd = True
            t_switch_to_pd = t_current
            print(f"INFO: Переключение на ПД-контроллер на шаге {i}, время t={t_current:.3f}с")
            # Опционально: можно сбросить интегратор ПИ-контроллера мотора при переключении,
            # чтобы избежать "скачка" из-за накопленной ошибки от предыдущего режима.
            # pi_motor_ctrl.reset_integral()

    # 1. Рассчитать желаемый момент tau_desired
    if active_controller_is_pd:
        tau_desired = pd_stabilizer.compute_control(
            current_value=current_theta,
            current_derivative=current_theta_dot,
            target_value=np.pi, # Цель - верхнее положение
            target_derivative=0.0 # Целевая скорость - 0
        )
        controller_mode_history[i] = 1 # Логируем режим ПД
    else:
        # EnergySwingUpController ожидает состояние [theta, theta_dot, motor_torque]
        temp_full_state_for_energy_ctrl = np.array([
            current_theta, 
            current_theta_dot, 
            current_motor_state[0]
        ])
        tau_desired = energy_ctrl.compute_control(temp_full_state_for_energy_ctrl, t_current)
        controller_mode_history[i] = 0 # Логируем режим Energy

    tau_desired_history[i, :] = tau_desired.flatten()

    # 2. Рассчитать управляющий сигнал 'a' от PIController
    #    PIController ожидает текущий момент мотора и tau_desired как цель
    current_motor_torque_value = current_motor_state[0] 
    control_a = pi_motor_ctrl.compute_control(
        current_state=np.array([current_motor_torque_value]), 
        target_state=tau_desired,
        t=t_current
    )
    control_a_history[i, :] = control_a.flatten()
    # pi_integral_error_history[i+1, :] = pi_motor_ctrl.get_integral_error().flatten() # Удалено
        
    # 3. Рассчитать производные состояния для маятника и двигателя
    # 3.1 Производные маятника
    #     Вход для маятника - это текущий реальный момент двигателя
    actual_torque_on_pendulum = current_motor_state[0]
    pendulum_derivatives = pendulum_actual_sys.get_state_derivative(
        control_input=actual_torque_on_pendulum, # Это torque для системы Pendulum
        state=current_pendulum_state, 
        t=t_current
    )
    
    # 3.2 Производные двигателя
    #     Вход для двигателя - это команда 'a'
    motor_derivatives = motor_actual_sys.get_state_derivative(
        u=control_a, # Это команда 'a' для системы MotorSystem
        state=current_motor_state,
        t=t_current
    )

    # Логируем производные в общий массив для совместимости графиков
    derivatives_history[i, 0:2] = pendulum_derivatives.flatten()
    derivatives_history[i, 2] = motor_derivatives.flatten()[0]
    
    # 4. Обновить состояние системы (метод Эйлера)
    current_pendulum_state = current_pendulum_state + pendulum_derivatives * dT
    current_motor_state = current_motor_state + motor_derivatives * dT
        
    # 5. Сохранить новое состояние и время
    pendulum_state_history[i+1, :] = current_pendulum_state
    motor_state_history[i+1, :] = current_motor_state
    time_history[i+1] = (i+1) * dT
    
    # 6. Рассчитать и сохранить энергии
    # Расчет энергий по формулам из EnergySwingUpController
    Ep_history[i+1] = m_pend * g_pend * l_pend * (1 - np.cos(current_pendulum_state[0]))
    Ek_history[i+1] = 0.5 * J_pend * current_pendulum_state[1]**2
    current_energy_history[i+1] = Ep_history[i+1] + Ek_history[i+1]

    if (i + 1) % (N_steps // 10) == 0:
        print(f"Симуляция: {((i + 1) * 100) / N_steps :.0f}% завершена")

print("Ручная симуляция завершена.")
if t_switch_to_pd > 0:
    print(f"Переключение на ПД-контроллер произошло в t = {t_switch_to_pd:.3f} с")
else:
    print("Переключения на ПД-контроллер не произошло.")

# --- Отрисовка результатов ---
img_save_dir = os.path.join(PARENT_OF_SCRIPTS_DIR, "img")
os.makedirs(img_save_dir, exist_ok=True)
plot_save_path = os.path.join(img_save_dir, "img_3_1_pendulum_swing_up_pd_sim.png") # Новое имя файла

# Явно отключаем использование полного LaTeX для избежания проблем с парсингом
plt.rcParams['text.usetex'] = False

print(f"Отрисовка результатов. Сохранение в: {plot_save_path}")

fig, axes = plt.subplots(2, 3, figsize=(20, 12)) # Изменено на 2x3
title_str = (
    f'Pendulum Swing-up with PD Stabilization & PI Motor Control\n' # Изменено имя системы
    f'm={m_pend}kg, l={l_pend}m, damp_p={damping_pend}, gamma_m={motor_gamma}s, u_max_swing={u_max_swing_up}Nm, alpha_motor={alpha_motor_desired:.0f}rad/s\n'
    f'PD_stab: Kp={Kp_pd_pend:.2f}, Kd={Kd_pd_pend:.2f}, alpha_stab={alpha_pend_stab:.2f} | Switch: ang_err<{angle_threshold_for_pd:.2f}rad, vel<{velocity_threshold_for_pd:.2f}rad/s'
)
fig.suptitle(title_str, fontsize=12)

# --- График 1 (axes[0,0]): Угол маятника и Угловая скорость ---
ax1 = axes[0,0]
color_angle = 'tab:blue'
ax1.set_xlabel('Time (s)')
ax1.set_ylabel('Angle (rad)', color=color_angle)
ax1.plot(time_history, pendulum_state_history[:, 0], label=r'$\theta$ (rad)', color=color_angle)
ax1.tick_params(axis='y', labelcolor=color_angle)
ax1.axhline(np.pi, color='r', linestyle='--', label=r'$\theta = \pi$')
ax1.axhline(-np.pi, color='r', linestyle='--', label=r'$\theta = -\pi$')
ax1.axhline(0, color='grey', linestyle='--')
ax1.grid(True, linestyle='--', alpha=0.7)

ax1b = ax1.twinx() # Создаем вторую ось Y, разделяющую ось X
color_velocity = 'tab:green'
ax1b.set_ylabel('Angular Velocity (rad/s)', color=color_velocity) 
ax1b.plot(time_history, pendulum_state_history[:, 1], label=r'$\dot{\theta}$ (rad/s)', color=color_velocity, linestyle=':')
ax1b.tick_params(axis='y', labelcolor=color_velocity)

ax1.set_title('Pendulum Angle & Angular Velocity')
# Собираем легенды с обеих осей
lines, labels = ax1.get_legend_handles_labels()
lines2, labels2 = ax1b.get_legend_handles_labels()
ax1.legend(lines + lines2, labels + labels2, loc='best')

# --- График 2 (axes[0,1] - БЫЛ [1,0]): Моменты (желаемый EnergySwingUp/PD, фактический моторный) ---
ax_moments = axes[0,1] # Новое расположение
ax_moments.plot(time_history[:-1], tau_desired_history[:, 0], label=r'$\tau_{des}$ (SwingUp/PD) (Nm)', linestyle='--')
ax_moments.plot(time_history, motor_state_history[:, 0], label=r'$\tau_m$ (Actual Motor Torque) (Nm)', color='sienna', linewidth=1.2)
if t_switch_to_pd > 0:
    ax_moments.axvline(t_switch_to_pd, color='lime', linestyle='-.', linewidth=1.5, label=f'PD Switch @ {t_switch_to_pd:.2f}s')
ax_moments.set_xlabel('Time (s)')
ax_moments.set_ylabel('Torque (Nm)')
ax_moments.set_title('Desired (Main Ctrl) vs Actual Motor Torque')
ax_moments.legend(loc='best')
ax_moments.grid(True)

# --- График 3 (axes[0,2] - БЫЛ [0,1]): Фазовый портрет маятника ---
ax_fp_pend = axes[0,2] # Новое расположение
points_pend = np.array([pendulum_state_history[:, 0], pendulum_state_history[:, 1]]).T.reshape(-1, 1, 2)
segments_pend = np.concatenate([points_pend[:-1], points_pend[1:]], axis=1)
norm_pend_time = plt.Normalize(time_history.min(), time_history.max())
lc_pend_colors = plt.cm.rainbow(norm_pend_time(time_history[:-1])) 

for j in range(len(segments_pend)):
    ax_fp_pend.plot(segments_pend[j,:,0], segments_pend[j,:,1], color=lc_pend_colors[j], linestyle='-', linewidth=1.5)

ax_fp_pend.set_xlabel(r'$\theta$ (rad)')
ax_fp_pend.set_ylabel(r'$\dot{\theta}$ (rad/s)')
ax_fp_pend.set_title('Pendulum Phase Portrait (colored by time)')
sm_pend = plt.cm.ScalarMappable(cmap=cm.rainbow, norm=norm_pend_time)
sm_pend.set_array([])
fig.colorbar(sm_pend, ax=ax_fp_pend, orientation='vertical', label='Time (s)')
ax_fp_pend.grid(True)
ax_fp_pend.scatter(pendulum_state_history[0, 0], pendulum_state_history[0, 1], 
            s=100, c='green', marker='o', label='Start', zorder=3)
ax_fp_pend.scatter(pendulum_state_history[-1, 0], pendulum_state_history[-1, 1], 
            s=100, c='red', marker='x', label='End', zorder=3)
legend_handles_ax_fp_pend = [
    Line2D([0], [0], marker='o', color='w', label='Start', markerfacecolor='green', markersize=7),
    Line2D([0], [0], marker='x', color='w', label='End', markerfacecolor='red', markeredgecolor='red', markersize=7)
]
ax_fp_pend.legend(handles=legend_handles_ax_fp_pend, loc='best')


# --- График 4 (axes[1,0] - БЫЛ [1,1]): Управляющий сигнал 'a' для двигателя и режим контроллера ---
ax_motor_cmd = axes[1,0] # Новое расположение
unit_a = motor_actual_sys.control_input_units[0] if hasattr(motor_actual_sys, 'control_input_units') else 'cmd_unit'
ax_motor_cmd.step(time_history[:-1], control_a_history[:, 0], where='post', label=f'Motor Command \'a\' ({unit_a})', color='red')
ax_motor_cmd.set_xlabel('Time (s)')
ax_motor_cmd.set_ylabel(f'Motor Command ({unit_a})', color='red') 
ax_motor_cmd.tick_params(axis='y', labelcolor='red')
ax_motor_cmd.set_title('Motor Command & Controller Mode')
ax_motor_cmd.grid(True)

ax_motor_cmd_b = ax_motor_cmd.twinx()
ax_motor_cmd_b.plot(time_history[:-1], controller_mode_history, label='Controller Mode (0=Energy, 1=PD)', color='blue', linestyle=':', drawstyle='steps-post')
ax_motor_cmd_b.set_ylabel('Controller Mode', color='blue')
ax_motor_cmd_b.tick_params(axis='y', labelcolor='blue')
ax_motor_cmd_b.set_yticks([0, 1])
ax_motor_cmd_b.set_yticklabels(['Energy', 'PD'])

# Собираем элементы для легенды, чтобы избежать дублирования PD Switch
handles1, labels1 = ax_motor_cmd.get_legend_handles_labels()
handles2, labels2 = ax_motor_cmd_b.get_legend_handles_labels()

# Добавляем линию PD Switch только один раз, если она есть
pd_switch_label = ''
pd_switch_handle = None
if t_switch_to_pd > 0:
    # Находим существующую линию axvline, чтобы не добавлять новую
    for line in ax_motor_cmd.lines:
        if line.get_label() == f'PD Switch @ {t_switch_to_pd:.2f}s':
            pd_switch_handle = line
            pd_switch_label = line.get_label()
            break
    if pd_switch_handle is None: # Если вдруг не нашли (не должно случиться)
        pd_switch_line = ax_motor_cmd.axvline(t_switch_to_pd, color='lime', linestyle='-.', linewidth=1.5, label=f'PD Switch @ {t_switch_to_pd:.2f}s')
        pd_switch_handle = pd_switch_line
        pd_switch_label = pd_switch_line.get_label()
    
    # Убедимся, что у команды 'a' есть своя метка
    if not handles1 or labels1[0] != f'Motor Command \'a\' ({unit_a})':
        # Если у ax_motor_cmd.step нет явной метки, возьмем ее из handles1[0]
        # или добавим ее к handles1, если она там отсутствует
        pass # Предполагается, что label уже есть у step

    all_handles = handles1 + handles2
    all_labels = labels1 + labels2
    # Удаляем дубликаты, если есть, особенно для PD Switch
    # Простое удаление дублирующихся pd_switch_label из handles2/labels2, если он там есть
    if pd_switch_label in labels2:
        idx_in_2 = labels2.index(pd_switch_label)
        handles2.pop(idx_in_2)
        labels2.pop(idx_in_2)
        all_handles = handles1 + handles2
        all_labels = labels1 + labels2
    elif pd_switch_label in labels1 and len(handles1) > 1: # если он в handles1 и там не только он
        # Не нужно ничего делать, если pd_switch_handle это единственное, что в handles1
        pass 

else: # Если нет переключения
    all_handles = handles1 + handles2
    all_labels = labels1 + labels2

ax_motor_cmd.legend(all_handles, all_labels, loc='lower right') # Изменено на 'upper right' и более аккуратное формирование легенды


# --- График 5 (axes[1,1] - БЫЛ [1,2]): Энергия маятника ---
ax_energy = axes[1,1] # Новое расположение
E_target_upright = energy_ctrl.E_target_upright_nominal 
ax_energy.plot(time_history, current_energy_history, label='Total Energy (E_tot)')
ax_energy.plot(time_history, Ep_history, label='Potential Energy (Ep)', linestyle=':')
ax_energy.plot(time_history, Ek_history, label='Kinetic Energy (Ek)', linestyle=':')
ax_energy.axhline(E_target_upright, color='r', linestyle='--', label=f'E_upright = {E_target_upright:.2f} J')
if hasattr(energy_ctrl, 'E_target_operational') and energy_ctrl.E_target_operational != E_target_upright:
    ax_energy.axhline(energy_ctrl.E_target_operational, color='magenta', linestyle='--', label=f'E_operational = {energy_ctrl.E_target_operational:.2f} J')

# Расчет и отображение приближенных энергетических порогов для переключения
# J_pend уже должен быть определен ранее в скрипте
# Минимальная потенциальная энергия в зоне переключения по углу (при theta = pi - angle_threshold_for_pd, скорость = 0)
E_pot_at_angle_thresh = m_pend * g_pend * l_pend * (1 - np.cos(np.pi - angle_threshold_for_pd))
ax_energy.axhline(E_pot_at_angle_thresh, color='darkgoldenrod', linestyle=':', linewidth=1.5, label=rf'Min E for $\Delta\theta_{{sw}}$ ({E_pot_at_angle_thresh:.2f} J)')

# Максимальная полная энергия для переключения (при theta = pi, скорость = velocity_threshold_for_pd)
E_kin_at_vel_thresh = 0.5 * J_pend * (velocity_threshold_for_pd**2)
E_total_max_for_switch = E_target_upright + E_kin_at_vel_thresh # E_target_upright это 2*m*g*l
ax_energy.axhline(E_total_max_for_switch, color='teal', linestyle=':', linewidth=1.5, label=rf'Max E for $\dot{{\theta}}_{{sw}}$ ({E_total_max_for_switch:.2f} J)')

ax_energy.set_xlabel('Time (s)')
ax_energy.set_ylabel('Energy (J)') # Возвращено описание оси
ax_energy.set_title('Pendulum Energy & Approx. Switch Thresholds') # Обновленный заголовок
ax_energy.legend(loc='best', fontsize='small') # Изменено расположение легенды
ax_energy.grid(True) # Возвращена сетка для линейной шкалы

# --- График 6 (axes[1,2] - БЫЛ [0,2]): Фазовый портрет мотора ---
ax_fp_motor = axes[1,2] # Новое расположение
time_for_mpp = time_history[:-1] 
m_torque_for_mpp = motor_state_history[:-1, 0] 
m_torque_dot_for_mpp = derivatives_history[:, 2] 

points_motor = np.array([m_torque_for_mpp, m_torque_dot_for_mpp]).T.reshape(-1, 1, 2)
segments_motor = np.concatenate([points_motor[:-1], points_motor[1:]], axis=1) 

norm_motor_time = plt.Normalize(time_for_mpp[:-1].min(), time_for_mpp[:-1].max())
lc_motor_colors = plt.cm.rainbow(norm_motor_time(time_for_mpp[:-1])) 

for j in range(len(segments_motor)):
    ax_fp_motor.plot(segments_motor[j,:,0], segments_motor[j,:,1], color=lc_motor_colors[j], linestyle='-', linewidth=1.5)

ax_fp_motor.set_xlabel(r'Motor Torque $\tau_m$ (Nm)')
ax_fp_motor.set_ylabel(r'Motor Torque Derivative $\dot{\tau}_m$ (Nm/s)')
ax_fp_motor.set_title('Motor Phase Portrait (colored by time)')
sm_motor = plt.cm.ScalarMappable(cmap=cm.rainbow, norm=norm_motor_time)
sm_motor.set_array([])
fig.colorbar(sm_motor, ax=ax_fp_motor, orientation='vertical', label='Time (s)')
ax_fp_motor.grid(True)
ax_fp_motor.scatter(m_torque_for_mpp[0], m_torque_dot_for_mpp[0], 
            s=100, c='green', marker='o', label='Start', zorder=3)
ax_fp_motor.scatter(m_torque_for_mpp[-1], m_torque_dot_for_mpp[-1], 
            s=100, c='red', marker='x', label='End', zorder=3)
legend_handles_ax_fp_motor = [
    Line2D([0], [0], marker='o', color='w', label='Start', markerfacecolor='green', markersize=7),
    Line2D([0], [0], marker='x', color='w', label='End', markerfacecolor='red', markeredgecolor='red', markersize=7)
]
ax_fp_motor.legend(handles=legend_handles_ax_fp_motor, loc='best')


plt.tight_layout(rect=[0, 0.03, 1, 0.95]) 
plt.savefig(plot_save_path)
print(f"График сохранен в: {plot_save_path}")
# plt.show()
plt.close(fig)

print(f"Скрипт {os.path.basename(__file__)} завершен.") 