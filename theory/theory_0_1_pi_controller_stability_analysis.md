# Theoretical Analysis: PI Controller for First-Order Motor

## 1. System and Controller Model

Consider a first-order motor model:

$$\dot{\tau}(t) = \frac{1}{\gamma}(-\tau(t) + u(t)) \quad (1)$$

where:
- $\tau(t)$ - motor torque (system state).
- $u(t)$ - control input (controller output).
- $\gamma > 0$ - motor time constant.

We want the torque $\tau(t)$ to track a desired reference value $\tau_{ref}$.
The tracking error is defined as:

$$e(t) = \tau_{ref} - \tau(t) \quad (2)$$

To eliminate the steady-state error observed with a P-controller, we introduce a PI controller. The control input $u(t)$ for a PI controller is:

$$u(t) = K_p e(t) + K_i \int_0^t e(\xi) d\xi \quad (3)$$

where:
- $K_p > 0$ - proportional gain.
- $K_i > 0$ - integral gain.

## 2. Closed-Loop System Equations

To obtain the closed-loop system equations, differentiate equation (3) with respect to time:

$$\dot{u}(t) = K_p \dot{e}(t) + K_i e(t) \quad (4)$$

From equation (2), it follows that $\dot{e}(t) = \dot{\tau}_{ref} - \dot{\tau}(t)$. For stability analysis around a steady state where $\tau_{ref}$ is constant, $\dot{\tau}_{ref} = 0$. Then:

$$\dot{e}(t) = -\dot{\tau}(t) \quad (5)$$

Substitute (5) into (4):

$$\dot{u}(t) = -K_p \dot{\tau}(t) + K_i e(t) \quad (6)$$

Now, rewrite the motor equation (1) as:

$$\gamma \dot{\tau}(t) + \tau(t) = u(t) \quad (7)$$

Differentiate (7) with respect to time:

$$\gamma \ddot{\tau}(t) + \dot{\tau}(t) = \dot{u}(t) \quad (8)$$

Now substitute the expression for $\dot{u}(t)$ from (6) into (8):

$$\gamma \ddot{\tau}(t) + \dot{\tau}(t) = -K_p \dot{\tau}(t) + K_i e(t)$$

Replace $e(t)$ from (2), considering $\tau_{ref} = const$:

$$\gamma \ddot{\tau}(t) + \dot{\tau}(t) = -K_p \dot{\tau}(t) + K_i (\tau_{ref} - \tau(t))$$

Rearrange the terms to obtain a differential equation with respect to $\tau(t)$:

$$\gamma \ddot{\tau}(t) + (1 + K_p) \dot{\tau}(t) + K_i \tau(t) = K_i \tau_{ref} \quad (9)$$

This is a second-order linear non-homogeneous differential equation with respect to $\tau(t)$.

## 3. Stability Analysis using Lyapunov's First Method

For stability analysis, consider the homogeneous part of equation (9):

$$\gamma \ddot{\tau}(t) + (1 + K_p) \dot{\tau}(t) + K_i \tau(t) = 0 \quad (10)$$

The characteristic equation for (10) is (replacing $\tau(t)$ with $e^{\lambda t}$):

$$\gamma \lambda^2 + (1 + K_p) \lambda + K_i = 0 \quad (11)$$

This is a quadratic equation of the form $a\lambda^2 + b\lambda + c = 0$, where:
- $a = \gamma$
- $b = 1 + K_p$
- $c = K_i$

The roots of the characteristic equation (system eigenvalues) are:

$$\lambda_{1,2} = \frac{-(1 + K_p) \pm \sqrt{(1 + K_p)^2 - 4 \gamma K_i}}{2 \gamma}$$

For asymptotic stability of the system, all coefficients of the characteristic polynomial must be positive (since $\gamma > 0$, this holds for $1+K_p > 0$ and $K_i > 0$, which we assume for a PI controller) and all roots $\lambda_{1,2}$ must have negative real parts.

The Routh-Hurwitz stability conditions for a second-order polynomial $a_2 \lambda^2 + a_1 \lambda + a_0 = 0$ (in our case $a_2 = \gamma, a_1 = 1+K_p, a_0 = K_i$) require all coefficients to have the same sign (positive, since $\gamma > 0$):
1. $\gamma > 0$ (given)
2. $1 + K_p > 0 \implies K_p > -1$. Since we choose $K_p > 0$, this condition is met.
3. $K_i > 0$ (we choose $K_i > 0$).

These conditions ensure that if the roots are real, they are negative, and if they are complex, their real part is negative.
Thus, for $K_p > 0$ and $K_i > 0$, the system will be stable.

## 4. Requirement for Equal and Negative Eigenvalues

We want the system to have two equal, real, negative eigenvalues. This corresponds to critical damping and ensures an aperiodic transient response (without oscillations) and fast convergence to $\tau_{ref}$.

For the roots to be equal, the discriminant of the characteristic equation (11) must be zero:

$$(1 + K_p)^2 - 4 \gamma K_i = 0 \quad (12)$$

In this case, both roots will be equal:

$$\lambda_1 = \lambda_2 = \lambda_0 = -\frac{1 + K_p}{2 \gamma} \quad (13)$$

Since we want these roots to be negative, and $\gamma > 0$, we need $1 + K_p > 0$, which is already met for $K_p > 0$.

From equation (12), we can express $K_i$ in terms of $K_p$ (or vice versa):

$$K_i = \frac{(1 + K_p)^2}{4 \gamma} \quad (14)$$

If we set a desired value for the equal roots, say $\lambda_0 = -\alpha$ (where $\alpha > 0$), then from (13):

$$-\alpha = -\frac{1 + K_p}{2 \gamma}$$
$$2 \gamma \alpha = 1 + K_p$$
$$K_p = 2 \gamma \alpha - 1 \quad (15)$$

For $K_p > 0$, it is necessary that $2 \gamma \alpha - 1 > 0$, i.e., $\alpha > \frac{1}{2\gamma}$.

Substituting $K_p$ from (15) into (14), we get $K_i$ in terms of $\alpha$:

$$K_i = \frac{(1 + (2 \gamma \alpha - 1))^2}{4 \gamma} = \frac{(2 \gamma \alpha)^2}{4 \gamma} = \frac{4 \gamma^2 \alpha^2}{4 \gamma} = \gamma \alpha^2 \quad (16)$$

**Thus, to obtain two equal negative eigenvalues $\lambda_0 = -\alpha$ (where $\alpha > \frac{1}{2\gamma}$), the PI controller gains should be chosen as follows:**

-   **$K_p = 2 \gamma \alpha - 1$**
-   **$K_i = \gamma \alpha^2$**

**Choosing $\alpha$:**
The parameter $\alpha$ determines how quickly the transient response decays (the larger $\alpha$, the faster). However, very large values of $\alpha$ will lead to large values of $K_p$ and $K_i$, which can cause control signal saturation or excite unmodeled high-frequency dynamics.
The choice of $\alpha$ is a trade-off between response speed and control effort.

## 5. Alternative Approach to Coefficient Selection (via Desired Polynomial)

If we want the characteristic polynomial of the closed-loop system (11) to match a desired polynomial $(\lambda - \lambda_0)^2 = \lambda^2 - 2\lambda_0 \lambda + \lambda_0^2 = 0$, then, dividing (11) by $\gamma$, we get:

$$\lambda^2 + \frac{1 + K_p}{\gamma} \lambda + \frac{K_i}{\gamma} = 0$$

Comparing coefficients with $\lambda^2 + 2\alpha \lambda + \alpha^2 = 0$ (where $\lambda_0 = -\alpha$):

1.  $\frac{1 + K_p}{\gamma} = 2\alpha \implies 1 + K_p = 2\alpha\gamma \implies K_p = 2\alpha\gamma - 1$
2.  $\frac{K_i}{\gamma} = \alpha^2 \implies K_i = \alpha^2\gamma$

The results match. 