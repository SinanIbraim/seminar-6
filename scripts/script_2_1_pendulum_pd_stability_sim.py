import numpy as np
import matplotlib.pyplot as plt
import os
import sys

# --- Настройка путей для импорта ---
SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
PARENT_OF_SCRIPTS_DIR = os.path.dirname(SCRIPT_DIR)
if PARENT_OF_SCRIPTS_DIR not in sys.path:
    sys.path.append(PARENT_OF_SCRIPTS_DIR)

# --- Импорты из src ---
try:
    from src.systems.pendulum import Pendulum
    from src.systems.motor_system import MotorSystem
    from src.controllers.pd_controller import PDController # Новый контроллер
    from src.controllers.pi_controller import PIController
except ImportError as e:
    print(f"Ошибка импорта: {e}. Убедитесь, что структура проекта верна и PYTHONPATH настроен.")
    exit()

# --- Параметры симуляции ---
dT = 0.005
T_sim = 10.0
N_steps = int(T_sim / dT)

# --- Параметры маятника ---
m_pend = 0.6  # Масса маятника (кг)
l_pend = 0.5  # Длина маятника до центра масс (м)
g_pend = 9.81 # Ускорение свободного падения (м/с^2)
damping_pend = 0.01 # Коэффициент вязкого трения маятника (Н*м*с/рад)
J_pend = m_pend * l_pend**2 # Момент инерции маятника

# --- Параметры ПД-регулятора для стабилизации маятника ---
# Цель: стабилизировать маятник в theta = pi
# Коэффициенты из теории для критического затухания (theory_2_1_pd_controller_pendulum_stability_en.md)
# K_d = 2*J*alpha, K_p = J*alpha^2 - mgl
# alpha > sqrt(g/l) для K_p > 0
sqrt_g_l = np.sqrt(g_pend / l_pend)
alpha_pend_stab = 1.5 * sqrt_g_l # Выбираем alpha для стабилизации (например, в 1.5 раза больше минимального)

Kp_pend = J_pend * alpha_pend_stab**2 - m_pend * g_pend * l_pend
Kd_pend = 2 * J_pend * alpha_pend_stab
tau_max_pd_output = 2.0 # Максимальный желаемый момент от ПД-регулятора маятника (Нм)

print(f"Параметры маятника: J={J_pend:.3f} kg*m^2, mgl={m_pend*g_pend*l_pend:.2f} Nm")
print(f"Параметры ПД-стабилизации маятника (цель: alpha_pend_stab={alpha_pend_stab:.2f} rad/s):")
print(f"  Kp_pend = {Kp_pend:.2f}, Kd_pend = {Kd_pend:.2f}")
if Kp_pend <= 0:
    print("ПРЕДУПРЕЖДЕНИЕ: Kp_pend <= 0. Это может быть неэффективно для стабилизации.")

# --- Параметры мотора ---
motor_gamma = 0.05  # Постоянная времени мотора (с)

# --- Параметры ПИ-регулятора для контура момента мотора ---
alpha_motor_track = 80.0  # Желаемое быстродействие контура момента (рад/с)
Kp_motor = 2 * motor_gamma * alpha_motor_track - 1
Ki_motor = motor_gamma * (alpha_motor_track**2)
integral_limit_motor = tau_max_pd_output * 1.5 # Ограничение интеграла, например, 150% от макс. момента ПД

print(f"Параметры ПИ-контроллера мотора: Kp_motor = {Kp_motor:.2f}, Ki_motor = {Ki_motor:.2f}, integral_limit = {integral_limit_motor:.2f}")

# --- Начальное состояние --- 
initial_theta = np.pi - 0.1 # Начальный угол маятника (чуть смещен от верхнего положения)
initial_theta_dot = 0.0     # Начальная угловая скорость маятника
initial_motor_torque = 0.0  # Начальный момент двигателя

initial_pendulum_state = np.array([initial_theta, initial_theta_dot])
initial_motor_state = np.array([initial_motor_torque])

# --- Инициализация систем и контроллеров ---
pendulum_sys = Pendulum(
    mass=m_pend,
    length=l_pend,
    gravity=g_pend,
    damping=damping_pend,
    initial_state=initial_pendulum_state.copy()
)
pendulum_sys.name = "Pendulum"
pendulum_sys.state_names = ['Angle theta', 'Ang. vel. theta_dot']
pendulum_sys.state_units = ['rad', 'rad/s']

motor_sys = MotorSystem(
    initial_state=initial_motor_state.copy(),
    gamma=motor_gamma
)

pd_pendulum_stabilizer = PDController(
    Kp=Kp_pend,
    Kd=Kd_pend,
    dt=dT,
    control_limits=(-tau_max_pd_output, tau_max_pd_output),
    name="Pendulum_PD_Stabilizer"
)

pi_motor_torque_ctrl = PIController(
    Kp=Kp_motor,
    Ki=Ki_motor,
    dt=dT,
    integral_limit=integral_limit_motor,
    initial_integral_error=np.array([0.0]),
    name="Motor_PI_Torque_Control"
)

# --- Подготовка для хранения истории ---
time_history = np.zeros(N_steps + 1)
pendulum_state_history = np.zeros((N_steps + 1, 2))
motor_state_history = np.zeros((N_steps + 1, 1))
tau_desired_pend_history = np.zeros((N_steps, 1)) # Выход ПД-регулятора маятника
control_a_history = np.zeros((N_steps, 1))      # Выход ПИ-регулятора мотора

# --- Начальные значения для истории ---
time_history[0] = 0.0
pendulum_state_history[0, :] = initial_pendulum_state
motor_state_history[0, :] = initial_motor_state

current_pendulum_state = initial_pendulum_state.copy()
current_motor_state = initial_motor_state.copy()

print(f"\nЗапуск симуляции стабилизации маятника: {N_steps} шагов, dT={dT}c, T_sim={T_sim}c")
# --- Основной цикл симуляции ---
for i in range(N_steps):
    t_current = i * dT

    # 1. ПД-регулятор маятника: вычислить желаемый момент tau_desired_pend
    # Цель: theta = pi, theta_dot = 0
    tau_desired_pend = pd_pendulum_stabilizer.compute_control(
        current_value=current_pendulum_state[0],       # current theta
        current_derivative=current_pendulum_state[1],  # current theta_dot
        target_value=np.pi,                            # target theta
        target_derivative=0.0                          # target theta_dot
    )
    tau_desired_pend_history[i, :] = tau_desired_pend.flatten()

    # 2. ПИ-регулятор момента двигателя: вычислить команду 'a'
    # Цель: current_motor_torque отслеживает tau_desired_pend
    control_a = pi_motor_torque_ctrl.compute_control(
        current_state=current_motor_state, # current motor torque [tau_m]
        target_state=tau_desired_pend,     # target motor torque
        t=t_current
    )
    control_a_history[i, :] = control_a.flatten()

    # 3. Обновить состояние систем
    # 3.1 Маятник (управляется фактическим моментом двигателя current_motor_state[0])
    pendulum_derivatives = pendulum_sys.get_state_derivative(
        control_input=current_motor_state[0],
        state=current_pendulum_state,
        t=t_current
    )
    current_pendulum_state = current_pendulum_state + pendulum_derivatives * dT

    # 3.2 Двигатель (управляется командой 'a')
    motor_derivatives = motor_sys.get_state_derivative(
        u=control_a,
        state=current_motor_state,
        t=t_current
    )
    current_motor_state = current_motor_state + motor_derivatives * dT

    # 4. Сохранить новое состояние и время
    time_history[i+1] = (i+1) * dT
    pendulum_state_history[i+1, :] = current_pendulum_state
    motor_state_history[i+1, :] = current_motor_state

    if (i + 1) % (N_steps // 20) == 0:
        print(f"Симуляция: {((i + 1) * 100) / N_steps :.0f}% завершена. Theta: {current_pendulum_state[0]:.3f} rad")

print("Симуляция завершена.")

# --- Отрисовка результатов ---
img_save_dir = os.path.join(PARENT_OF_SCRIPTS_DIR, "img")
os.makedirs(img_save_dir, exist_ok=True)
plot_save_path = os.path.join(img_save_dir, "img_2_1_pendulum_pd_stability_sim.png")

plt.rcParams['text.usetex'] = False # Отключаем LaTeX для избежания проблем с парсингом
plt.rcParams['font.size'] = 10

fig, axes = plt.subplots(2, 2, figsize=(15, 10))
fig.suptitle(f"PD Stabilization of Inverted Pendulum with Motor Dynamics\n" +
             f"m={m_pend}kg, l={l_pend}m, J={J_pend:.3f}kgm^2, b_pend={damping_pend:.3f}Ns/m, gamma_m={motor_gamma}s\n"+
             f"PD_pend: Kp={Kp_pend:.2f}, Kd={Kd_pend:.2f} (alpha_stab={alpha_pend_stab:.2f}) => tau_max={tau_max_pd_output}Nm\n"+
             f"PI_motor: Kp={Kp_motor:.2f}, Ki={Ki_motor:.2f} (alpha_track={alpha_motor_track:.1f})")

time_plot = time_history[:-1] # Для данных длиной N_steps

# 1. Угол маятника и ошибка угла
ax1 = axes[0,0]
ax1.plot(time_history, pendulum_state_history[:, 0], label=r'$\theta$ (rad)')
ax1.plot(time_history, np.pi - pendulum_state_history[:, 0], label=r'Error ($\pi - \theta$) (rad)', linestyle=':')
ax1.axhline(np.pi, color='grey', linestyle='--', label=r'$\theta = \pi$')
ax1.set_xlabel('Time (s)')
ax1.set_ylabel('Angle (rad)')
ax1.set_title('Pendulum Angle and Error')
ax1.legend()
ax1.grid(True)

# 2. Угловая скорость маятника
ax2 = axes[0,1]
ax2.plot(time_history, pendulum_state_history[:, 1], label=r'$\dot{\theta}$ (rad/s)', color='purple')
ax2.set_xlabel('Time (s)')
ax2.set_ylabel('Angular Velocity (rad/s)')
ax2.set_title('Pendulum Angular Velocity')
ax2.legend()
ax2.grid(True)

# 3. Моменты: желаемый ПД-регулятором и фактический момент двигателя
ax3 = axes[1,0]
ax3.plot(time_plot, tau_desired_pend_history[:, 0], label=r'$\tau_{des,pend}$ (PD output) (Nm)', linestyle='--')
ax3.plot(time_history, motor_state_history[:, 0], label=r'$\tau_m$ (Actual Motor Torque) (Nm)')
ax3.set_xlabel('Time (s)')
ax3.set_ylabel('Torque (Nm)')
ax3.set_title('Desired (PD) vs Actual Motor Torque')
ax3.legend()
ax3.grid(True)

# 4. Управляющий сигнал 'a' для двигателя
ax4 = axes[1,1]
unit_a = motor_sys.control_input_units[0] if hasattr(motor_sys, 'control_input_units') else 'cmd_unit'
ax4.step(time_plot, control_a_history[:, 0], where='post', label=f'Motor Command \'a\' ({unit_a})', color='red')
ax4.set_xlabel('Time (s)')
ax4.set_ylabel(f'Motor Command \'a\' ({unit_a})')
ax4.set_title('Motor Control Command \'a\'')
ax4.legend()
ax4.grid(True)

plt.tight_layout(rect=[0, 0.03, 1, 0.93]) # Отрегулировать, чтобы заголовок поместился
plt.savefig(plot_save_path)
print(f"\nГрафик сохранен в: {plot_save_path}")
# plt.show()
plt.close(fig)

print("Скрипт script_2_1_pendulum_pd_stability_sim.py завершен.") 