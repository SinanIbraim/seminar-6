import numpy as np
import matplotlib.pyplot as plt
import os
import sys

# --- Настройка путей для импорта --- 
SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
# PARENT_OF_SCRIPTS_DIR указывает на каталог, содержащий папку src (например, .../seminar_7)
PARENT_OF_SCRIPTS_DIR = os.path.dirname(SCRIPT_DIR)

if PARENT_OF_SCRIPTS_DIR not in sys.path:
    sys.path.append(PARENT_OF_SCRIPTS_DIR)

# --- Импорты из src --- 
try:
    from src.systems.motor_system import MotorSystem
    from src.controllers.proportional_controller import ProportionalController
    from src.simulator import Simulator
    from src.plotter import Plotter # Импортируем здесь, т.к. используется для кастомного графика
except ImportError as e:
    print(f"Ошибка импорта: {e}")
    print(f"SCRIPT_DIR: {SCRIPT_DIR}")
    print(f"PARENT_OF_SCRIPTS_DIR (должен содержать src): {PARENT_OF_SCRIPTS_DIR}")
    print(f"sys.path: {sys.path}")
    exit()

# --- Параметры симуляции --- 
dT = 0.01  # Шаг времени
T_sim = 10.0  # Общее время симуляции
N_steps = int(T_sim / dT) # Количество шагов

# --- Параметры системы (двигателя) --- 
initial_tau = 0.0  # Начальный крутящий момент
motor_gamma = 0.2    # Постоянная времени двигателя

# --- Параметры контроллера --- 
Kp = 15.0 # Коэффициент усиления П-контроллера

# --- Функция для генерации меандра --- 
def meander_target_func(t: float, amplitude: float = 1.0, period: float = 2.0) -> np.ndarray:
    """
    Генерирует меандр (прямоугольную волну) как целевое значение.

    Args:
        t (float): Текущее время.
        amplitude (float): Амплитуда меандра.
        period (float): Период меандра.

    Returns:
        np.ndarray: Целевое значение [target_tau].
    """
    if (t % period) < (period / 2):
        return np.array([amplitude])
    else:
        return np.array([-amplitude])

# --- Основная функция --- 
def main():
    print(f"Запуск скрипта из: {SCRIPT_DIR}")

    # 1. Создание системы
    motor = MotorSystem(initial_state=np.array([initial_tau]), gamma=motor_gamma)

    # 2. Создание контроллера
    controller = ProportionalController(
        Kp=Kp, 
        target_func=lambda t: meander_target_func(t, amplitude=1.5, period=4.0)
    )

    # 3. Создание симулятора
    simulator = Simulator(
        system=motor,
        controller=controller,
        dt=dT,
        num_steps=N_steps
    )

    # 4. Запуск симуляции
    print("Запуск симуляции...")
    simulator.run()
    print("Симуляция завершена.")

    # 5. Отображение результатов
    time_history, state_history, control_history, _, _ = simulator.get_results()
    target_torque_history = np.array([meander_target_func(t, amplitude=1.5, period=4.0) for t in time_history]) # Перегенерируем для графика

    # img_save_dir должен быть относительно PARENT_OF_SCRIPTS_DIR (т.е. .../seminar_7/img)
    img_save_dir = os.path.join(PARENT_OF_SCRIPTS_DIR, "img") 
    if not os.path.exists(img_save_dir):
        os.makedirs(img_save_dir)
    plot_save_path = os.path.join(img_save_dir, "img_0_2_motor_meander_target.png")

    print(f"Отрисовка результатов. Сохранение в: {plot_save_path}")
    
    fig, axes = plt.subplots(3, 1, figsize=(18, 10), sharex=True)
    fig.suptitle(f'{motor.name} with Proportional Control (Kp={Kp}, gamma={motor.gamma}) - Meander Target', fontsize=16)

    axes[0].plot(time_history, state_history[:, 0], label=f'{motor.state_names[0]} ({motor.state_units[0]})')
    axes[0].plot(time_history, target_torque_history[:, 0], label='Target Torque (Nm)', linestyle='--', color='red')
    axes[0].set_ylabel(f'{motor.state_names[0]} ({motor.state_units[0]})')
    axes[0].legend(loc='best')
    axes[0].grid(True)

    if control_history is not None:
        time_for_control = time_history[:-1] if len(time_history) == control_history.shape[0] + 1 else time_history
        if len(time_for_control) != control_history.shape[0]:
             print(f"Warning: Длина времени для управления ({len(time_for_control)}) не совпадает с control_history ({control_history.shape[0]})")
        axes[1].step(time_for_control, control_history[:, 0], where='post', label=f'{motor.control_input_names[0]} ({motor.control_input_units[0]})', color='green')
        axes[1].set_ylabel(f'{motor.control_input_names[0]} ({motor.control_input_units[0]})')
        axes[1].legend(loc='best')
        axes[1].grid(True)

    error_history = target_torque_history[:,0] - state_history[:,0] # Убедимся, что вычитаем 1D массивы
    axes[2].plot(time_history, error_history, label='Error (Target - Actual Torque) (Nm)', color='purple') # error_history теперь 1D
    axes[2].set_ylabel('Error (Nm)')
    axes[2].set_xlabel('Time (s)')
    axes[2].legend(loc='best')
    axes[2].grid(True)

    plt.tight_layout(rect=[0, 0, 1, 0.96])
    
    if plot_save_path:
        os.makedirs(os.path.dirname(plot_save_path), exist_ok=True)
        plt.savefig(plot_save_path)
        print(f"График сохранен в: {plot_save_path}")
    else:
        plt.show()
    plt.close(fig)

    print("Скрипт завершен.")

if __name__ == "__main__":
    main() 