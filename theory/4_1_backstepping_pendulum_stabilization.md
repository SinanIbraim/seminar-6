# Backstepping Control for Inverted Pendulum with Motor Dynamics

## 1. Introduction

This section extends the backstepping control design to stabilize a simple pendulum at its upper equilibrium point $(\theta = \pi, \dot{\theta} = 0)$ by explicitly considering the first-order dynamics of the motor that generates the torque.

## 2. System Model (with Motor Dynamics)

The pendulum dynamics are given by:
$$ J \ddot{\theta} = \tau_m - mgl \sin{\theta} - D \dot{\theta} $$
where $\tau_m$ is the actual torque applied by the motor.

The motor dynamics are modeled as a first-order system:
$$ \dot{\tau}_m = \frac{1}{\gamma}(-\tau_m + a) $$
where $a$ is the control input to the motor (e.g., voltage or current command), and $\gamma$ is the motor time constant.

Our goal is to stabilize $(\theta, \dot{\theta}) = (\pi, 0)$, which implies $\tau_m$ should also go to an appropriate steady-state value (which will be 0 at equilibrium).

Define the error coordinates and state variables for backstepping:
1.  $x_1 = e_1 = \theta - \pi$ (Pendulum angle error)
2.  $x_2 = e_2 = \dot{\theta}$ (Pendulum angular velocity)
3.  $x_3 = \tau_m$ (Actual motor torque)

The system equations in terms of these states are:
$$ \dot{x}_1 = x_2 $$
$$ \dot{x}_2 = \frac{1}{J}(x_3 + mgl \sin(x_1) - D x_2) \quad (\text{since } \sin(x_1+\pi) = -\sin(x_1)) $$
$$ \dot{x}_3 = \frac{1}{\gamma}(-x_3 + a) $$

## 3. Step-by-Step Controller Design (Backstepping - 3 Steps)

### Step 1: Stabilizing $x_1$ (Angle Error)

Consider the $\dot{x}_1 = x_2$ subsystem. We treat $x_2$ as a virtual control.
Lyapunov function candidate: $L_1 = \frac{1}{2}x_1^2$.
Its derivative: $\dot{L}_1 = x_1 \dot{x}_1 = x_1 x_2$.
Define the desired virtual control $x_{2d} = -c_1 x_1$, with $c_1 > 0$.
If $x_2 = x_{2d}$, then $\dot{L}_1 = -c_1 x_1^2 \le 0$.

Define the first error variable $\bar{e}_2 = x_2 - x_{2d} = x_2 + c_1 x_1$.
Then $x_2 = \bar{e}_2 - c_1 x_1$, and $\dot{x}_1 = \bar{e}_2 - c_1 x_1$.
So, $\dot{L}_1 = x_1(\bar{e}_2 - c_1 x_1) = x_1 \bar{e}_2 - c_1 x_1^2$.

### Step 2: Stabilizing $\bar{e}_2$ (Effective Velocity Error)

Now, consider $x_3$ (motor torque $\tau_m$) as the virtual control for the $(x_1, \bar{e}_2)$ subsystem.
Augmented Lyapunov function: $L_2 = L_1 + \frac{1}{2}\bar{e}_2^2 = \frac{1}{2}x_1^2 + \frac{1}{2}\bar{e}_2^2$.
Derivative: $\dot{L}_2 = \dot{L}_1 + \bar{e}_2 \dot{\bar{e}}_2 = x_1 \bar{e}_2 - c_1 x_1^2 + \bar{e}_2 \dot{\bar{e}}_2$.
We need $\dot{\bar{e}}_2 = \dot{x}_2 + c_1 \dot{x}_1 = \dot{x}_2 + c_1 x_2$.
$$ \dot{\bar{e}}_2 = \frac{1}{J}(x_3 + mgl \sin(x_1) - D x_2) + c_1 x_2 $$
Substitute into $\dot{L}_2$:
$$ \dot{L}_2 = -c_1 x_1^2 + \bar{e}_2 \left[ x_1 + \frac{1}{J}(x_3 + mgl \sin(x_1) - D x_2) + c_1 x_2 \right] $$
Define the desired virtual control $x_{3d}$ (desired motor torque $\tau_{des}$) such that the term in brackets is $-c_2 \bar{e}_2$, with $c_2 > 0$.
$$ x_1 + \frac{1}{J}(x_{3d} + mgl \sin(x_1) - D x_2) + c_1 x_2 = -c_2 \bar{e}_2 $$
Solving for $x_{3d}$:
$$ x_{3d} = -J(x_1 + c_1 x_2 + c_2 \bar{e}_2) - mgl \sin(x_1) + D x_2 $$
Substituting $\bar{e}_2 = x_2 + c_1 x_1$:
$$ x_{3d} = \tau_{des} = -J(1+c_1c_2)x_1 - (J(c_1+c_2)-D)x_2 - mgl \sin(x_1) $$
This is the same desired torque as derived in the 2-step version.

Define the second error variable $\bar{e}_3 = x_3 - x_{3d} = \tau_m - \tau_{des}$.
Then $x_3 = \bar{e}_3 + x_{3d}$. The term in brackets in $\dot{L}_2$ becomes:
$$ x_1 + \frac{1}{J}( ( \bar{e}_3 + x_{3d}) + mgl \sin(x_1) - D x_2) + c_1 x_2 = -c_2 \bar{e}_2 + \frac{1}{J}\bar{e}_3 $$
So, $\dot{L}_2 = -c_1 x_1^2 - c_2 \bar{e}_2^2 + \frac{1}{J}\bar{e}_2 \bar{e}_3$.

### Step 3: Stabilizing $\bar{e}_3$ (Torque Error)

Now, we design the actual control input $a$ to make $\bar{e}_3 \to 0$.
Augmented Lyapunov function (scaling the $\bar{e}_3$ term by $\gamma$ for simpler algebra later):
$$ L_3 = L_2 + \frac{\gamma}{2}\bar{e}_3^2 = \frac{1}{2}x_1^2 + \frac{1}{2}\bar{e}_2^2 + \frac{\gamma}{2}\bar{e}_3^2 $$
Derivative: $\dot{L}_3 = \dot{L}_2 + \gamma \bar{e}_3 \dot{\bar{e}}_3 = -c_1 x_1^2 - c_2 \bar{e}_2^2 + \frac{1}{J}\bar{e}_2 \bar{e}_3 + \gamma \bar{e}_3 \dot{\bar{e}}_3$.
We need $\dot{\bar{e}}_3 = \dot{x}_3 - \dot{x}_{3d}$.
$$ \dot{x}_3 = \frac{1}{\gamma}(-x_3 + a) $$
$$ \dot{x}_{3d} = \frac{\partial x_{3d}}{\partial x_1}\dot{x}_1 + \frac{\partial x_{3d}}{\partial x_2}\dot{x}_2 = \frac{\partial x_{3d}}{\partial x_1}x_2 + \frac{\partial x_{3d}}{\partial x_2} \left( \frac{1}{J}(x_3 + mgl \sin(x_1) - D x_2) \right) $$
Let $K_{31} = -J(1+c_1c_2)$ and $K_{32} = -(J(c_1+c_2)-D)$. So, $x_{3d} = K_{31}x_1 + K_{32}x_2 - mgl \sin(x_1)$.
Then $\frac{\partial x_{3d}}{\partial x_1} = K_{31} - mgl \cos(x_1)$ and $\frac{\partial x_{3d}}{\partial x_2} = K_{32}$.

Substitute $\dot{\bar{e}}_3$ into $\dot{L}_3$:
$$ \dot{L}_3 = -c_1 x_1^2 - c_2 \bar{e}_2^2 + \bar{e}_3 \left[ \frac{1}{J}\bar{e}_2 + \gamma \left( \frac{1}{\gamma}(-x_3 + a) - \dot{x}_{3d} \right) \right] $$
$$ \dot{L}_3 = -c_1 x_1^2 - c_2 \bar{e}_2^2 + \bar{e}_3 \left[ \frac{1}{J}\bar{e}_2 - x_3 + a - \gamma \dot{x}_{3d} \right] $$
To make $\dot{L}_3 \le 0$, we choose the term in brackets to be $-c_3 \bar{e}_3$, with $c_3 > 0$.
$$ \frac{1}{J}\bar{e}_2 - x_3 + a - \gamma \dot{x}_{3d} = -c_3 \bar{e}_3 $$
Solving for the actual control input $a$:
$$ a = x_3 - \frac{1}{J}\bar{e}_2 + \gamma \dot{x}_{3d} - c_3 \bar{e}_3 $$

## 4. Resulting Control Law for $a$

The control input $a$ to the motor is derived from $ a = x_3 - \frac{1}{J}\bar{e}_2 + \gamma \dot{x}_{3d} - c_3 \bar{e}_3 $. 
Substituting $\bar{e}_2 = x_2 + c_1 x_1$ and $\bar{e}_3 = x_3 - x_{3d}$, this becomes:
$$ a = (1-c_3)x_3 - \frac{c_1}{J}x_1 - \frac{1}{J}x_2 + c_3 x_{3d} + \gamma \dot{x}_{3d} $$
where $x_1 = \theta - \pi$, $x_2 = \dot{\theta}$, $x_3 = \tau_m$ (the motor torque), and the terms $x_{3d}$ (desired torque) and $\dot{x}_{3d}$ (its time derivative) are functions of these states:
- $x_{3d} = -J(1+c_1c_2)x_1 - (J(c_1+c_2)-D)x_2 - mgl \sin(x_1)$.
- $\dot{x}_{3d} = (-J(1+c_1c_2) - mgl \cos(x_1))x_2 - \frac{J(c_1+c_2)-D}{J}(x_3 + mgl \sin(x_1) - D x_2)$.

Explicitly substituting these expressions for $x_{3d}$ and $\dot{x}_{3d}$ into the equation for $a$, we obtain the full control law as a function of states $(x_1, x_2, x_3)$:
$$ 
\begin{aligned}
a(x_1, x_2, x_3) ={} & (1-c_3)x_3 - \frac{c_1}{J}x_1 - \frac{1}{J}x_2 \\
& + c_3 \Big( -J(1+c_1c_2)x_1 - (J(c_1+c_2)-D)x_2 - mgl \sin(x_1) \Big) \\
& + \gamma \Big( (-J(1+c_1c_2) - mgl \cos(x_1))x_2 \\
& \qquad - \frac{J(c_1+c_2)-D}{J}(x_3 + mgl \sin(x_1) - D x_2) \Big)
\end{aligned}
$$ 
This highlights the nonlinear and coupled nature of the resulting control input.

## 5. Connection to Controller in Code

The backstepping controller synthesizes a command $a$ for the motor system.
1.  The term $\tau_{des}$ is precisely the desired torque that the PD controller (stabilizing the pendulum) in the original code aims to compute based on $e_1$ and $e_2$.
2.  The additional terms in the control law $a$ ($x_3 - \frac{1}{J}\bar{e}_2 + \gamma \dot{x}_{3d} - c_3 \bar{e}_3$) serve to drive the actual motor torque $\tau_m$ to this $\tau_{des}$, while accounting for the motor dynamics ($\gamma$) and the coupling effects captured by $\dot{\tau}_{des}$ and $\bar{e}_2$. This is a more sophisticated way to achieve torque tracking than a standard linear PI controller for the motor torque, as it is derived from a Lyapunov stability argument for the entire 3rd-order system.
    - The term $x_3 (= \tau_m)$ is a feedforward of current torque.
    - $-c_3 \bar{e}_3 = -c_3 (\tau_m - \tau_{des})$ is a proportional feedback on the torque error.
    - $\gamma \dot{\tau}_{des}$ is a feedforward of the desired torque rate.
    - $-\frac{1}{J}\bar{e}_2$ is a coupling term from the pendulum velocity error dynamics.

This is effectively a nonlinear state-feedback controller for the motor that includes feedforward terms and ensures overall system stability.

## 6. Stability Proof

With the control law for $a$ chosen as:
$$ a = x_3 - \frac{1}{J}\bar{e}_2 + \gamma \dot{x}_{3d} - c_3 \bar{e}_3 $$
The derivative of the Lyapunov function $L_3$ becomes:
$$ \dot{L}_3 = -c_1 x_1^2 - c_2 \bar{e}_2^2 - c_3 \bar{e}_3^2 $$
Since $c_1, c_2, c_3 > 0$, $\dot{L}_3 \le 0$ for all $(x_1, \bar{e}_2, \bar{e}_3)$.
$\dot{L}_3 = 0$ if and only if $x_1=0$, $\bar{e}_2=0$, and $\bar{e}_3=0$.
- If $x_1=0$, then $e_1 = \theta - \pi = 0 \implies \theta = \pi$.
- If $x_1=0$ and $\bar{e}_2=0$, then $x_2 = \bar{e}_2 - c_1 x_1 = 0 \implies e_2 = \dot{\theta} = 0$.
- If $x_1=0, x_2=0$ and $\bar{e}_3=0$, then $x_3 = \bar{e}_3 + x_{3d}$. Since $x_1=0, x_2=0$, $x_{3d} = -J(1+c_1c_2)(0) - (J(c_1+c_2)-D)(0) - mgl \sin(0) = 0$. So, $x_3 = \tau_m = 0$.

The set where $\dot{L}_3=0$ is the single point $(x_1, x_2, x_3) = (0,0,0)$, which corresponds to the desired equilibrium $(\theta=\pi, \dot{\theta}=0, \tau_m=0)$.
By LaSalle's invariance principle, the system trajectories converge to this equilibrium, making it asymptotically stable. 