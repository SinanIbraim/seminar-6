import numpy as np
import matplotlib.pyplot as plt
import matplotlib.cm as cm
import os
import sys

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
    from src.controllers.hierarchical_pendulum_controller import HierarchicalPendulumController
except ImportError as e:
    print(f"Ошибка импорта: {e}. Убедитесь, что структура проекта верна и PYTHONPATH настроен.")
    print(f"SCRIPT_DIR: {SCRIPT_DIR}")
    print(f"PARENT_OF_SCRIPTS_DIR (должен содержать src): {PARENT_OF_SCRIPTS_DIR}")
    print(f"sys.path: {sys.path}")
    exit()

# --- Параметры симуляции --- 
dT = 0.005  # Шаг времени (уменьшим для большей точности ручного Эйлера)
T_sim = 10.0  # Общее время симуляции
N_steps = int(T_sim / dT) # Количество шагов

# --- Параметры маятника ---
m_pend = 0.6  # Масса маятника (кг)
l_pend = 0.5  # Длина маятника до центра масс (м)
g_pend = 9.81 # Ускорение свободного падения (м/с^2)
damping_pend = 0.00 # Коэффициент вязкого трения маятника (Н*м*с/рад) - НОВЫЙ ПАРАМЕТР

# --- Параметры мотора ---
motor_gamma = 0.05  # Постоянная времени мотора (с)

# --- Начальное состояние системы ---
# [theta, theta_dot, motor_torque]
initial_state_vector = np.array([0.01, 0.0, 0.0]) # Маятник почти внизу, покоится, момент 0
initial_pendulum_state = initial_state_vector[0:2]
initial_motor_state = np.array([initial_state_vector[2]]) # Должен быть 1D массив для MotorSystem

# --- Параметры контроллера энергетической раскачки ---
u_max_swing_up = 1.5  # Максимальный ЖЕЛАЕМЫЙ момент от EnergySwingUpController (Нм)
# Этот момент будет целью для ПИ-регулятора мотора.
s2_deadband_swing_up = 0.05 # Порог для угловой скорости в EnergySwingUpController
energy_target_factor = 1.02 # Немного "перепрыгнуть" целевую энергию ( > 1.0)

# --- Параметры ПИ-регулятора для контура момента мотора ---
# Цель: обеспечить быстрое отслеживание tau_desired моментом tau_m
# Используем формулы Kp = 2*alpha*gamma - 1, Ki = alpha^2*gamma
# alpha_motor определяет скорость сходимости контура момента. 
# Чем больше alpha_motor, тем быстрее реакция, но больше риск неустойчивости или шумов.
alpha_motor_desired = 80.0  # Желаемое расположение полюсов (-alpha_motor) для контура момента (рад/с)

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

# HierarchicalPendulumController больше не используется в этом скрипте напрямую
# hierarchical_ctrl = HierarchicalPendulumController(
#     energy_controller=energy_ctrl,
#     pi_motor_controller=pi_motor_ctrl,
#     pendulum_state_indices=(0, 1),
#     motor_torque_state_index=2 
# )

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
J_pend = m_pend * l_pend**2 
Ep_history[0] = m_pend * g_pend * l_pend * (1 - np.cos(current_pendulum_state[0]))
Ek_history[0] = 0.5 * J_pend * current_pendulum_state[1]**2
current_energy_history[0] = Ep_history[0] + Ek_history[0]
# pi_integral_error_history[0, :] = pi_motor_ctrl.get_integral_error().flatten() # Удалено


print(f"Запуск ручной симуляции: {N_steps} шагов, dT={dT}c, T_sim={T_sim}c")
# --- Основной цикл симуляции (ручной) ---
for i in range(N_steps):
    t_current = i * dT
    
    # 1. Рассчитать желаемый момент tau_desired от EnergySwingUpController
    #    EnergySwingUpController ожидает состояние [theta, theta_dot, motor_torque]
    temp_full_state_for_energy_ctrl = np.array([
        current_pendulum_state[0], 
        current_pendulum_state[1], 
        current_motor_state[0]
    ])
    tau_desired = energy_ctrl.compute_control(temp_full_state_for_energy_ctrl, t_current)
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

# --- Отрисовка результатов ---
img_save_dir = os.path.join(PARENT_OF_SCRIPTS_DIR, "img")
os.makedirs(img_save_dir, exist_ok=True)
plot_save_path = os.path.join(img_save_dir, "img_1_1_pendulum_swing_up_manual_sim_v2.png") # Новое имя файла

# Явно отключаем использование полного LaTeX для избежания проблем с парсингом
plt.rcParams['text.usetex'] = False

print(f"Отрисовка результатов. Сохранение в: {plot_save_path}")

fig, axes = plt.subplots(2, 3, figsize=(20, 12)) # Изменено на 2x3
title_str = (
    f'Pendulum & Motor with Energy Swing-up & PI Control (Manual Sim V2)\n' # Изменено имя системы
    f'm={m_pend}kg, l={l_pend}m, b_pend={damping_pend}, gamma_motor={motor_gamma}s, u_max_swing={u_max_swing_up}Nm, alpha_motor={alpha_motor_desired:.0f}rad/s' # Добавлен b_pend
)
fig.suptitle(title_str, fontsize=14)

# Данные для графиков (чтобы не срезать последний элемент time_history для некоторых)
time_plot = time_history[:-1] # Для данных длиной N_steps

# 1. Момент двигателя, цель + производная управления
ax1 = axes[0,0]
ax1.plot(time_history, motor_state_history[:, 0], label=f'Actual tau_m ({motor_actual_sys.state_units[0]})') # Убран LaTeX
ax1.plot(time_plot, tau_desired_history[:, 0], label='Desired tau_des (Nm)', linestyle='--') # Убран LaTeX
ax1.set_xlabel('Time (s)')
ax1.set_ylabel('Motor Torque (Nm)')
ax1.set_title('Motor Torque: Actual vs Desired') # Добавлен заголовок
ax1.grid(True)
ax1.legend(loc='upper right') # Убрана вторая легенда и ax1_twin

# 2. Управление двигателем 'a'
ax2 = axes[0,1]
unit_a = motor_actual_sys.control_input_units[0] if hasattr(motor_actual_sys, 'control_input_units') else 'cmd_unit'
line_a, = ax2.step(time_plot, control_a_history[:, 0], where='post', label=f'Motor Cmd a ({unit_a})', color='red') # Сохраняем объект линии
ax2.set_xlabel('Time (s)')
ax2.set_ylabel(f'Motor Command a ({unit_a})', color='red') # Метка для основной оси
ax2.tick_params(axis='y', labelcolor='red')
ax2.grid(True)

# Создаем вторую ось Y для tau_desired
ax2_twin = ax2.twinx()
line_tau_des, = ax2_twin.plot(time_plot, tau_desired_history[:, 0], label='Energy Ctrl Target Tau (Nm)', color='blue', linestyle='--') # Сохраняем объект линии
ax2_twin.set_ylabel('Energy Ctrl Target Tau (Nm)', color='blue')  # Метка для второй оси
ax2_twin.tick_params(axis='y', labelcolor='blue')
ax2_twin.grid(True, linestyle=':', alpha=0.7) # Добавляем сетку для второй оси

ax2.set_title('Motor Command \'a\' and Energy Controller Target')
# Объединяем легенды
lines = [line_a, line_tau_des]
labels = [l.get_label() for l in lines]
ax2.legend(lines, labels, loc='upper right')

# 3. Угол маятника + скорость маятника
ax3 = axes[0,2]
ax3.plot(time_history, pendulum_state_history[:, 0], label=f'{pendulum_actual_sys.state_names[0]} ({pendulum_actual_sys.state_units[0]})') # Используются обновленные имена без LaTeX
ax3.axhline(np.pi, color='grey', linestyle='--', label='theta = pi') # Убран LaTeX
ax3.axhline(-np.pi, color='grey', linestyle='--')
ax3.set_xlabel('Time (s)')
ax3.set_ylabel(f'{pendulum_actual_sys.state_names[0]} ({pendulum_actual_sys.state_units[0]})') # Используются обновленные имена
ax3.set_title('Pendulum State: Angle and Angular Velocity') # Добавлен заголовок

ax3_twin = ax3.twinx()
ax3_twin.plot(time_history, pendulum_state_history[:, 1], label=f'{pendulum_actual_sys.state_names[1]} ({pendulum_actual_sys.state_units[1]})', color='purple', linestyle=':') # Используются обновленные имена
ax3_twin.set_ylabel(f'{pendulum_actual_sys.state_names[1]} ({pendulum_actual_sys.state_units[1]})', color='purple') # Используются обновленные имена
ax3_twin.tick_params(axis='y', labelcolor='purple')

lines, labels = ax3.get_legend_handles_labels()
lines2, labels2 = ax3_twin.get_legend_handles_labels()
ax3.legend(lines + lines2, labels + labels2, loc='upper right')

# Убедимся, что на ax3 (угол маятника) есть сетка
axes[0,2].grid(True)

# 4. Энергии (потенциальная, кинетическая, общая, целевая)
ax4 = axes[1,0]
ax4.plot(time_history, Ep_history, label='Potential Energy $E_p$ (J)')
ax4.plot(time_history, Ek_history, label='Kinetic Energy $E_k$ (J)', linestyle='-.')
ax4.plot(time_history, current_energy_history, label='Total Energy $E_{total}$ (J)', linestyle='-')
ax4.axhline(energy_ctrl.E_target_operational, color='r', linestyle='--', label=f'$E_{{target,op}}={energy_ctrl.E_target_operational:.2f} J')
ax4.set_xlabel('Time (s)')
ax4.set_ylabel('Energy (J)')
ax4.set_title('System Energies') # Добавлен заголовок
ax4.legend(loc='center right')
ax4.grid(True)

# 5. Фазовый портрет движка (tau_m vs tau_m_dot)
ax5 = axes[1,1]
tau_m_values = motor_state_history[:N_steps, 0]
tau_m_dot_values = derivatives_history[:, 2]

# scatter5 = ax5.scatter(tau_m_values, tau_m_dot_values, c=time_plot[:N_steps], cmap=cm.rainbow, s=10, label='Motor Phase Trajectory') # Изменено на scatter, c=time_plot
ax5.plot(tau_m_values, tau_m_dot_values, color='orange', linewidth=0.8, label='Motor Phase Trajectory') # Возвращено к plot
ax5.scatter(tau_m_values[0], tau_m_dot_values[0], marker='o', color='blue', s=50, zorder=3, label='Start')
ax5.scatter(tau_m_values[-1], tau_m_dot_values[-1], marker='x', color='red', s=50, zorder=3, label='End')
ax5.set_xlabel(f'Motor Torque tau_m ({motor_actual_sys.state_units[0]})')
ax5.set_ylabel(f'Motor Torque Derivative tau_m_dot ({motor_actual_sys.state_units[0]}/s)')
ax5.set_title('Motor Phase Portrait')
ax5.legend(loc='upper right')
ax5.grid(True)

# 6. Фазовый портрет маятника (theta vs theta_dot)
ax6 = axes[1,2]
theta_normalized_for_phase_plot = (pendulum_state_history[:, 0] + np.pi) % (2 * np.pi) - np.pi
scatter6 = ax6.scatter(theta_normalized_for_phase_plot, pendulum_state_history[:, 1], c=time_history, cmap=cm.rainbow, s=10, label='Pendulum Phase Trajectory') # Изменено на scatter, c=time_history
ax6.scatter(theta_normalized_for_phase_plot[0], pendulum_state_history[0, 1], marker='o', color='blue', s=50, zorder=3, label='Start')
ax6.scatter(theta_normalized_for_phase_plot[-1], pendulum_state_history[-1, 1], marker='x', color='red', s=50, zorder=3, label='End')
ax6.set_xlabel(f'{pendulum_actual_sys.state_names[0]} (rad)')
ax6.set_ylabel(f'{pendulum_actual_sys.state_names[1]} (rad/s)')
ax6.set_title('Pendulum Phase Portrait (theta_dot vs theta)')
ax6.axvline(np.pi, color='grey', linestyle=':', label='Upright theta=pi')
ax6.axvline(-np.pi, color='grey', linestyle=':')
ax6.set_xlim([-np.pi - 0.2, np.pi + 0.2])
ax6.legend(loc='upper right')
ax6.grid(True)

# Добавляем один colorbar для времени, относящийся к ax6
cbar = fig.colorbar(scatter6, ax=ax6, label='Time (s)', shrink=0.9, aspect=25, pad=0.08) # Изменен ax на ax6, скорректированы параметры

# # 7. Интегральная ошибка ПИ-контроллера # Удалена вся секция
# ax7 = axes[2,0] 
# ax7.plot(time_history, pi_integral_error_history[:, 0], label='PI Integral Error e_int_PI') 
# ax7.set_xlabel('Time (s)')
# ax7.set_ylabel('Integral Error Value')
# ax7.set_title('PI Controller Integral Error')
# ax7.legend(loc='upper right')
# ax7.grid(True)

# # Скрыть неиспользуемые оси, если они есть (для сетки 3x3, если только 7 графиков) # Удалено
# if axes.shape == (3,3):
#     axes[2,1].set_visible(False)
#     axes[2,2].set_visible(False)

plt.tight_layout(rect=[0, 0.03, 1, 0.95]) 
plt.savefig(plot_save_path)
print(f"График сохранен в: {plot_save_path}")
# plt.show()
plt.close(fig)

print("Скрипт script_1_1_pendulum_swing_up_manual_sim.py завершен.") 