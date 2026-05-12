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
    from src.systems.motor_system import MotorSystem
    from src.controllers.pi_controller import PIController # Используем новый ПИ-контроллер
    from src.simulator import Simulator
    from src.plotter import Plotter 
except ImportError as e:
    print(f"Ошибка импорта: {e}. Убедитесь, что структура проекта верна и PYTHONPATH настроен.")
    print(f"SCRIPT_DIR: {SCRIPT_DIR}")
    print(f"PARENT_OF_SCRIPTS_DIR (должен содержать src): {PARENT_OF_SCRIPTS_DIR}")
    print(f"sys.path: {sys.path}")
    exit()

# --- Параметры симуляции --- 
dT = 0.01  # Шаг времени
T_sim = 10.0  # Общее время симуляции
N_steps = int(T_sim / dT) # Количество шагов

# --- Параметры системы (двигателя) --- 
initial_tau = 0.0
motor_gamma = 0.2  # Постоянная времени двигателя (такая же, как в script_0_2 и теории)

# --- Расчет коэффициентов ПИ-контроллера на основе теории --- 
# Выбираем alpha (желаемое расположение полюсов lambda = -alpha)
# Условие: alpha > 1 / (2 * motor_gamma)
alpha_desired = 5 # Можно поэкспериментировать с этим значением

# Проверка условия для alpha
alpha_min_limit = 1 / (2 * motor_gamma)
if alpha_desired <= alpha_min_limit:
    print(f"Предупреждение: Выбранное alpha_desired ({alpha_desired}) <= {alpha_min_limit:.2f}. "
          f"Это может привести к Kp <= 0. Рекомендуется alpha > {alpha_min_limit:.2f}.")

# Рассчитываем коэффициенты
Kp_calculated = 2 * motor_gamma * alpha_desired - 1
Ki_calculated = motor_gamma * (alpha_desired**2)

print(f"Параметры двигателя: gamma = {motor_gamma}")
print(f"Выбранное alpha для размещения полюсов: {alpha_desired}")
print(f"Рассчитанные коэффициенты ПИ-контроллера: Kp = {Kp_calculated:.4f}, Ki = {Ki_calculated:.4f}")

# --- Функция для генерации меандра (такая же, как в script_0_2) --- 
def meander_target_func(t: float, amplitude: float = 1.5, period: float = 4.0) -> np.ndarray:
    if (t % period) < (period / 2):
        return np.array([amplitude])
    else:
        return np.array([-amplitude])

# --- Основная функция --- 
def main():
    print(f"Запуск скрипта: {os.path.basename(__file__)}")

    motor = MotorSystem(initial_state=np.array([initial_tau]), gamma=motor_gamma)
    
    # Используем PIController
    controller = PIController(
        Kp=Kp_calculated,
        Ki=Ki_calculated,
        dt=dT, # Передаем шаг времени в контроллер
        target_func=lambda t: meander_target_func(t, amplitude=1.5, period=4.0),
        integral_limit=10.0 # Опционально: небольшой лимит для интегратора
    )

    simulator = Simulator(
        system=motor,
        controller=controller,
        dt=dT,
        num_steps=N_steps
    )

    print("Запуск симуляции...")
    simulator.run()
    print("Симуляция завершена.")

    time_history, state_history, control_history, _, _ = simulator.get_results()
    target_torque_history = np.array([meander_target_func(t, amplitude=1.5, period=4.0) for t in time_history])

    img_save_dir = os.path.join(PARENT_OF_SCRIPTS_DIR, "img")
    os.makedirs(img_save_dir, exist_ok=True)
    plot_save_path = os.path.join(img_save_dir, "img_0_3_motor_pi_control_meander.png")

    print(f"Отрисовка результатов. Сохранение в: {plot_save_path}")
    
    fig, axes = plt.subplots(3, 1, figsize=(12, 10), sharex=True)
    title_str = (
        rf'{motor.name} with PI Control ($\gamma={motor_gamma:.1f}, \alpha={alpha_desired:.1f}$)'
        f'$K_p={Kp_calculated:.2f}, K_i={Ki_calculated:.2f}$) - Meander Target'
    )
    fig.suptitle(title_str, fontsize=14)

    # 1. Состояние (крутящий момент) и Цель
    axes[0].plot(time_history, state_history[:, 0], label=f'{motor.state_names[0]} ({motor.state_units[0]})')
    axes[0].plot(time_history, target_torque_history[:, 0], label='Target Torque (Nm)', linestyle='--', color='red')
    axes[0].set_ylabel(f'{motor.state_names[0]} ({motor.state_units[0]})')
    axes[0].legend(loc='best')
    axes[0].grid(True)

    # 2. Управляющее воздействие
    if control_history is not None:
        time_for_control = time_history[:-1] if len(time_history) == control_history.shape[0] + 1 else time_history
        axes[1].step(time_for_control, control_history[:, 0], where='post', label=f'{motor.control_input_names[0]} ({motor.control_input_units[0]})', color='green')
        axes[1].set_ylabel(f'{motor.control_input_names[0]} ({motor.control_input_units[0]})')
        axes[1].legend(loc='best')
        axes[1].grid(True)

    # 3. Ошибка управления
    error_history = target_torque_history[:,0] - state_history[:,0]
    axes[2].plot(time_history, error_history, label='Error (Target - Actual Torque) (Nm)', color='purple')
    axes[2].set_ylabel('Error (Nm)')
    axes[2].set_xlabel('Time (s)')
    axes[2].legend(loc='best')
    axes[2].grid(True)

    plt.tight_layout(rect=[0, 0.02, 1, 0.95]) # Скорректировано для более длинного заголовка
    
    plt.savefig(plot_save_path)
    print(f"График сохранен в: {plot_save_path}")
    plt.close(fig)

    print("Скрипт завершен.")

if __name__ == "__main__":
    main() 