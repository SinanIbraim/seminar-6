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
    from src.systems.pendulum import Pendulum
    from src.systems.motor_system import MotorSystem
except ImportError as e:
    print(f"Ошибка импорта: {e}. Убедитесь, что структура проекта верна и PYTHONPATH настроен.")
    sys.exit()

# --- Параметры симуляции --- 
dT = 0.005  # Шаг времени
T_sim = 2.0  # Общее время симуляции
N_steps = int(T_sim / dT) # Количество шагов

# --- Параметры маятника ---
m_pend = 0.6  # Масса маятника (кг)
l_pend = 0.5  # Длина маятника до центра масс (м)
g_pend = 9.81 # Ускорение свободного падения (м/с^2)
D_pend = 0.01 # Коэффициент вязкого трения маятника (Н*м*с/рад)
J_pend = m_pend * l_pend**2 # Момент инерции маятника

# --- Параметры мотора ---
motor_gamma = 0.05  # Постоянная времени мотора (с)
u_max_motor = 10.0  # Максимальный момент мотора (Нм)

# --- Параметры Backstepping контроллера ---
# c1, c2 определяют стабилизацию маятника (через x_3d)
# c3 определяет стабилизацию контура момента
alpha_param = 1.5 * np.sqrt(g_pend / l_pend) # Желаемая "скорость" для маятника
if alpha_param <= 1.0:
    print(f"ПРЕДУПРЕЖДЕНИЕ: alpha_param ({alpha_param:.2f}) должен быть > 1 для c1 > 0. Увеличьте alpha_param.")
    alpha_param = 1.1 # Минимальное значение для c1 > 0
c1 = alpha_param - 1.0
c2 = alpha_param + 1.0
c3 = 20.0  # Настроечный коэффициент для контура момента

print(f"Параметры Backstepping: c1={c1:.2f}, c2={c2:.2f}, c3={c3:.2f} (alpha_param={alpha_param:.2f})")

# --- Начальное состояние системы ---
# x1 = theta - pi, x2 = theta_dot, x3 = motor_torque
initial_theta = np.pi - 0.5 # Начальный угол (небольшое отклонение от верхнего положения)
initial_theta_dot = 0.0     # Начальная угловая скорость
initial_motor_torque = 0.0  # Начальный момент мотора

initial_pendulum_state = np.array([initial_theta, initial_theta_dot])
initial_motor_state = np.array([initial_motor_torque])

# --- Инициализация систем ---
pendulum_sys = Pendulum(
    mass=m_pend,
    length=l_pend,
    gravity=g_pend,
    damping=D_pend, 
    initial_state=initial_pendulum_state.copy()
)
pendulum_sys.name = "Pendulum"
pendulum_sys.state_names = ['Angle theta', 'Ang. vel. theta_dot']
pendulum_sys.state_units = ['rad', 'rad/s']

motor_sys = MotorSystem(
    initial_state=initial_motor_state.copy(),
    gamma=motor_gamma
)

# --- Подготовка для хранения истории ---
time_history = np.zeros(N_steps + 1)
pendulum_state_history = np.zeros((N_steps + 1, 2)) # [theta, theta_dot]
motor_state_history = np.zeros((N_steps + 1, 1))    # [motor_torque]
control_a_history = np.zeros(N_steps)               # Команда 'a'
tau_desired_history = np.zeros(N_steps)             # x_3d (tau_des)
tau_desired_dot_history = np.zeros(N_steps)         # dot(x_3d) (dot_tau_des)
current_energy_history = np.zeros(N_steps + 1)
motor_torque_derivative_history = np.zeros(N_steps) # dot(tau_m)

# --- Начальные значения ---
current_pendulum_state = initial_pendulum_state.copy()
current_motor_torque = initial_motor_torque # Это скаляр

pendulum_state_history[0, :] = current_pendulum_state
motor_state_history[0, 0] = current_motor_torque
time_history[0] = 0.0

# Расчет начальных энергий
Ep_history = np.zeros_like(time_history)
Ek_history = np.zeros_like(time_history)
Ep_history[0] = m_pend * g_pend * l_pend * (1 - np.cos(current_pendulum_state[0]))
Ek_history[0] = 0.5 * J_pend * current_pendulum_state[1]**2
current_energy_history[0] = Ep_history[0] + Ek_history[0]

print(f"Запуск симуляции Backstepping: {N_steps} шагов, dT={dT}c, T_sim={T_sim}c")
# --- Основной цикл симуляции ---
for i in range(N_steps):
    t_current = i * dT
    
    # Текущие состояния маятника и момент мотора
    theta = current_pendulum_state[0]
    theta_dot = current_pendulum_state[1]
    tau_m = current_motor_torque # Скаляр
    
    # 1. Переменные ошибки и состояния для backstepping
    x1 = theta - np.pi  # Ошибка угла
    x2 = theta_dot      # Угловая скорость (ошибка скорости, т.к. цель 0)
    x3 = tau_m          # Момент мотора
    
    # 2. Вычисление x_3d (tau_des) - желаемый момент от шага 2 backstepping
    # x_3d = -J(1+c1*c2)x1 - (J(c1+c2)-D)x2 - mgl*sin(x1)
    term_x1_x3d = -J_pend * (1 + c1 * c2) * x1
    term_x2_x3d = -(J_pend * (c1 + c2) - D_pend) * x2
    term_sin_x3d = -m_pend * g_pend * l_pend * np.sin(x1)
    x_3d = term_x1_x3d + term_x2_x3d + term_sin_x3d
    tau_desired_history[i] = x_3d
    
    # 3. Вычисление dot(x_3d) (dot_tau_des)
    # dot(x_3d) = (d_x3d/d_x1)*dot(x1) + (d_x3d/d_x2)*dot(x2)
    # d_x3d/d_x1 = -J(1+c1*c2) - mgl*cos(x1)
    # d_x3d/d_x2 = -(J(c1+c2)-D)
    # dot(x1) = x2
    # dot(x2) = (1/J)*(x3 + mgl*sin(x1) - D*x2)
    
    partial_x3d_dx1 = -J_pend * (1 + c1 * c2) - m_pend * g_pend * l_pend * np.cos(x1)
    partial_x3d_dx2 = -(J_pend * (c1 + c2) - D_pend)
    
    dot_x1 = x2
    dot_x2_val = (1 / J_pend) * (x3 + m_pend * g_pend * l_pend * np.sin(x1) - D_pend * x2)
    
    dot_x_3d = partial_x3d_dx1 * dot_x1 + partial_x3d_dx2 * dot_x2_val
    tau_desired_dot_history[i] = dot_x_3d
    
    # 4. Вычисление управляющей команды 'a' для мотора (из шага 3 backstepping)
    # a = (1-c3)*x3 - (c1/J)*x1 - (1/J)*x2 + c3*x_3d + gamma*dot_x_3d
    # (Формула из теории: a = x3 - (1/J)*ebar2 + gamma*dot_x3d - c3*ebar3)
    # ebar2 = x2 + c1*x1
    # ebar3 = x3 - x_3d
    # a = x3 - (1/J)*(x2 + c1*x1) + gamma*dot_x_3d - c3*(x3 - x_3d)
    # a = (1-c3)*x3 - (c1/J)*x1 - (1/J)*x2 + c3*x_3d + gamma*dot_x_3d

    term1_a = (1 - c3) * x3
    term2_a = -(c1 / J_pend) * x1
    term3_a = -(1 / J_pend) * x2
    term4_a = c3 * x_3d
    term5_a = motor_gamma * dot_x_3d
    control_a = term1_a + term2_a + term3_a + term4_a + term5_a
    control_a = np.clip(control_a, -u_max_motor, u_max_motor) # Ограничение момента
    control_a_history[i] = control_a
    
    # 5. Рассчитать производные состояния для маятника и двигателя
    pendulum_derivatives = pendulum_sys.get_state_derivative(
        control_input=np.array([x3]), # Момент мотора x3 действует на маятник
        state=current_pendulum_state, 
        t=t_current
    )
    motor_derivatives = motor_sys.get_state_derivative(
        u=np.array([control_a]), # Команда 'a' действует на мотор
        state=np.array([current_motor_torque]), # current_motor_torque - скаляр
        t=t_current
    )
    motor_torque_derivative_history[i] = motor_derivatives[0] # Сохраняем производную момента
        
    # 6. Обновить состояние системы (метод Эйлера)
    current_pendulum_state = current_pendulum_state + pendulum_derivatives * dT
    current_motor_torque = current_motor_torque + motor_derivatives[0] * dT 
        
    # 7. Сохранить новое состояние и время
    pendulum_state_history[i+1, :] = current_pendulum_state
    motor_state_history[i+1, 0] = current_motor_torque 
    time_history[i+1] = (i+1) * dT
    
    # 8. Рассчитать и сохранить энергии
    Ep_history[i+1] = m_pend * g_pend * l_pend * (1 - np.cos(current_pendulum_state[0]))
    Ek_history[i+1] = 0.5 * J_pend * current_pendulum_state[1]**2
    current_energy_history[i+1] = Ep_history[i+1] + Ek_history[i+1]

    if (i + 1) % (N_steps // 10) == 0:
        print(f"Симуляция: {((i + 1) * 100) / N_steps :.0f}% завершена")

print("Симуляция Backstepping завершена.")

# --- Отрисовка результатов ---
img_save_dir = os.path.join(PARENT_OF_SCRIPTS_DIR, "img")
os.makedirs(img_save_dir, exist_ok=True)
plot_save_path = os.path.join(img_save_dir, "img_4_1_pendulum_backstepping_sim.png")

plt.rcParams['text.usetex'] = False
print(f"Отрисовка результатов. Сохранение в: {plot_save_path}")

fig, axes = plt.subplots(2, 3, figsize=(20, 12))
title_str = (
    f'Pendulum Stabilization with 3-Step Backstepping Control\n'
    f'm={m_pend}kg, l={l_pend}m, D_p={D_pend}, J_p={J_pend:.3f}, gamma_m={motor_gamma}s\n'
    f'Backstepping: c1={c1:.2f}, c2={c2:.2f}, c3={c3:.2f}, Motor Torque Limit: {u_max_motor} Nm'
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
ax1.grid(True, linestyle='--', alpha=0.7)

ax1b = ax1.twinx()
color_velocity = 'tab:green'
ax1b.set_ylabel('Angular Velocity (rad/s)', color=color_velocity) 
ax1b.plot(time_history, pendulum_state_history[:, 1], label=r'$\dot{\theta}$ (rad/s)', color=color_velocity, linestyle=':')
ax1b.tick_params(axis='y', labelcolor=color_velocity)
ax1.set_title('Pendulum Angle & Angular Velocity')
lines, labels = ax1.get_legend_handles_labels()
lines2, labels2 = ax1b.get_legend_handles_labels()
ax1.legend(lines + lines2, labels + labels2, loc='best')

# --- График 2 (axes[0,1]): Моменты (желаемый Backstepping, фактический моторный) ---
ax_moments = axes[0,1]
ax_moments.plot(time_history[:-1], tau_desired_history, label=r'$\tau_{des}$ (Backstepping) (Nm)', linestyle='--')
ax_moments.plot(time_history, motor_state_history[:, 0], label=r'$\tau_m$ (Actual Motor Torque) (Nm)', color='sienna', linewidth=1.2)
ax_moments.set_xlabel('Time (s)')
ax_moments.set_ylabel('Torque (Nm)')
ax_moments.set_title('Desired (Backstepping) vs Actual Motor Torque')
ax_moments.legend(loc='best')
ax_moments.grid(True)

# --- График 3 (axes[0,2]): Фазовый портрет маятника ---
ax_fp_pend = axes[0,2]
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

# --- График 4 (axes[1,0]): Управляющий сигнал 'a' для двигателя ---
ax_motor_cmd = axes[1,0]
unit_a = motor_sys.control_input_units[0] if hasattr(motor_sys, 'control_input_units') else 'cmd_unit'
ax_motor_cmd.step(time_history[:-1], control_a_history, where='post', label=f'Motor Command \'a\' ({unit_a})', color='red')
ax_motor_cmd.set_xlabel('Time (s)')
ax_motor_cmd.set_ylabel(f'Motor Command ({unit_a})') 
ax_motor_cmd.set_title('Motor Command \'a\' (Backstepping)')
ax_motor_cmd.grid(True)
ax_motor_cmd.legend(loc='best')

# --- График 5 (axes[1,1]): Энергия маятника ---
ax_energy = axes[1,1]
E_target_upright = m_pend * g_pend * l_pend * (1 - np.cos(np.pi)) # 2*m*g*l
ax_energy.plot(time_history, current_energy_history, label='Total Energy (E_tot)')
ax_energy.plot(time_history, Ep_history, label='Potential Energy (Ep)', linestyle=':')
ax_energy.plot(time_history, Ek_history, label='Kinetic Energy (Ek)', linestyle=':')
ax_energy.axhline(E_target_upright, color='r', linestyle='--', label=f'E_upright = {E_target_upright:.2f} J')
ax_energy.set_xlabel('Time (s)')
ax_energy.set_ylabel('Energy (J)')
ax_energy.set_title('Pendulum Energy')
ax_energy.legend(loc='best', fontsize='small')
ax_energy.grid(True)

# --- График 6 (axes[1,2]): Фазовый портрет мотора ---
ax_fp_motor = axes[1,2]
time_for_mpp = time_history[:-1] 
m_torque_for_mpp = motor_state_history[:-1, 0] 
m_torque_dot_for_mpp = motor_torque_derivative_history # dot(tau_m)

points_motor = np.array([m_torque_for_mpp, m_torque_dot_for_mpp]).T.reshape(-1, 1, 2)
segments_motor = np.concatenate([points_motor[:-1], points_motor[1:]], axis=1) 
norm_motor_time = plt.Normalize(time_for_mpp.min(), time_for_mpp.max()) # Use .min()/.max() on actual data
lc_motor_colors = plt.cm.rainbow(norm_motor_time(time_for_mpp)) # color for each segment

for j in range(len(segments_motor)): # Plot segments
    ax_fp_motor.plot(segments_motor[j,:,0], segments_motor[j,:,1], color=lc_motor_colors[j], linestyle='-', linewidth=1.5)

ax_fp_motor.set_xlabel(r'Motor Torque $\tau_m$ (Nm)')
ax_fp_motor.set_ylabel(r'Motor Torque Derivative $\dot{\tau}_m$ (Nm/s)')
ax_fp_motor.set_title('Motor Phase Portrait (colored by time)')
sm_motor = plt.cm.ScalarMappable(cmap=cm.rainbow, norm=norm_motor_time)
sm_motor.set_array([]) # Important for colorbar
fig.colorbar(sm_motor, ax=ax_fp_motor, orientation='vertical', label='Time (s)')
ax_fp_motor.grid(True)
if len(m_torque_for_mpp) > 0 and len(m_torque_dot_for_mpp) > 0: # Проверка, что массивы не пусты
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