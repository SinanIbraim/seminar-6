# Stability Analysis: PD Controller for Inverted Pendulum (Upright Position)

## 1. System Model (Pendulum)

Consider a simple mathematical pendulum without damping, whose equation of motion is:

$$J \ddot{\theta}(t) - mgl \sin(\theta(t)) = \tau(t) \quad (1)$$

where:
- $\theta(t)$ - angle of deviation of the pendulum from the vertical (downwards, $\theta=0$).
- $J$ - moment of inertia of the pendulum about the pivot point. For a rod with mass $m$ at the end and length $l$ (mass of the rod is negligible), $J = ml^2$.
- $m$ - mass of the pendulum.
- $g$ - acceleration due to gravity.
- $l$ - length of the pendulum (distance from the pivot point to the center of mass).
- $\tau(t)$ - control torque applied to the pendulum.

We want to stabilize the pendulum in the upright equilibrium position, i.e., $\theta_{ref} = \pi$.

## 2. Linearization at the Upright Equilibrium Position

For stability analysis in the vicinity of the upright position $\theta_{eq} = \pi$, introduce a small deviation variable $\delta\theta(t) = \theta(t) - \pi$.
Then $\theta(t) = \pi + \delta\theta(t)$, and $\ddot{\theta}(t) = \ddot{\delta\theta}(t)$.

Substitute into equation (1):
$$J \ddot{\delta\theta}(t) - mgl \sin(\pi + \delta\theta(t)) = \tau(t)$$

Use the trigonometric identity $\sin(\pi + x) = -\sin(x)$.
$$J \ddot{\delta\theta}(t) + mgl \sin(\delta\theta(t)) = \tau(t)$$

For small deviations $\delta\theta$, $\sin(\delta\theta) \approx \delta\theta$.
Then the linearized equation of motion of the pendulum in the vicinity of $\theta = \pi$ is:

$$J \ddot{\delta\theta}(t) + mgl \delta\theta(t) = \tau(t) \quad (2)$$

## 3. PD Controller

The control error is defined as $e(t) = \theta_{ref} - \theta(t)$.
Since $\theta_{ref} = \pi$ and $\theta(t) = \pi + \delta\theta(t)$, the error in terms of $\delta\theta$ is:
$e(t) = \pi - (\pi + \delta\theta(t)) = -\delta\theta(t)$.

The derivative of the error is: $\dot{e}(t) = -\dot{\delta\theta}(t)$.

The PD controller law is:
$$\tau(t) = K_p e(t) + K_d \dot{e}(t) \quad (3)$$
Substitute the expressions for $e(t)$ and $\dot{e}(t)$:
$$\tau(t) = K_p (-\delta\theta(t)) + K_d (-\dot{\delta\theta}(t))$$
$$\tau(t) = -K_p \delta\theta(t) - K_d \dot{\delta\theta}(t) \quad (4)$$

Here $K_p > 0$ and $K_d > 0$ are the proportional and derivative gains, respectively.

## 4. Closed-Loop System Equation

Substitute the control input $\tau(t)$ from (4) into the linearized pendulum equation (2):

$$J \ddot{\delta\theta}(t) + mgl \delta\theta(t) = -K_p \delta\theta(t) - K_d \dot{\delta\theta}(t)$$

Rearrange the terms to obtain a homogeneous differential equation with respect to $\delta\theta(t)$:

$$J \ddot{\delta\theta}(t) + K_d \dot{\delta\theta}(t) + (mgl + K_p) \delta\theta(t) = 0 \quad (5)$$

## 5. Characteristic Equation

For stability analysis using Lyapunov's first method (based on the roots of the characteristic equation), replace $\delta\theta(t)$ with $e^{\lambda t}$ in equation (5):

$$J \lambda^2 e^{\lambda t} + K_d \lambda e^{\lambda t} + (mgl + K_p) e^{\lambda t} = 0$$

Dividing by $e^{\lambda t}$ (which is non-zero), we get the characteristic equation:

$$J \lambda^2 + K_d \lambda + (mgl + K_p) = 0 \quad (6)$$

This is a quadratic equation of the form $a\lambda^2 + b\lambda + c = 0$, where:
- $a = J$
- $b = K_d$
- $c = mgl + K_p$

## 6. Stability Conditions

For asymptotic stability of the system (i.e., for $\delta\theta(t) \to 0$ as $t \to \infty$), all roots $\lambda_{1,2}$ of the characteristic equation (6) must have negative real parts.

According to the Routh-Hurwitz stability criterion for a second-order polynomial $a\lambda^2 + b\lambda + c = 0$, all coefficients must have the same sign (positive, since $J = ml^2 > 0$).
1. $J > 0$ (true by definition).
2. $K_d > 0$ (requirement for the PD controller).
3. $mgl + K_p > 0$. Since $m, g, l > 0$, this condition means $K_p > -mgl$. Typically, $K_p$ is chosen to be positive, so this condition is also met.

These conditions ($K_d > 0$ and $mgl + K_p > 0$) ensure that if the roots are real, they are negative, and if they are complex, their real part is negative.
Thus, for $K_d > 0$ and $K_p > -mgl$ (in practice $K_p > 0$), the system will be stable in the linearized approximation.

## 7. Critical Damping

We want the system to have two equal, real, negative eigenvalues (roots of the characteristic equation). This corresponds to critical damping and ensures an aperiodic transient response without oscillations to the state $\delta\theta = 0$.

For the roots to be equal, the discriminant $D = b^2 - 4ac$ of the characteristic equation (6) must be zero:

$$D = K_d^2 - 4 J (mgl + K_p) = 0 \quad (7)$$

In this case, both roots will be equal:

$$\lambda_1 = \lambda_2 = \lambda_0 = -\frac{K_d}{2J} \quad (8)$$

Since we want these roots to be negative, and $J > 0$, we need $K_d > 0$, which is already a stability condition.

Let the desired value for the equal roots be $\lambda_0 = -\alpha$, where $\alpha > 0$. Then from (8):

$$-\alpha = -\frac{K_d}{2J}$$
$$K_d = 2 J \alpha \quad (9)$$

Now substitute $K_d$ from (9) into the zero discriminant condition (7):

$$(2 J \alpha)^2 - 4 J (mgl + K_p) = 0$$
$$4 J^2 \alpha^2 = 4 J (mgl + K_p)$$

Since $J > 0$, we can divide by $4J$:

$$J \alpha^2 = mgl + K_p$$
From this, we express $K_p$:

$$K_p = J \alpha^2 - mgl \quad (10)$$

For $K_p > 0$ (which is often a practical requirement, although formally $K_p > -mgl$ is sufficient), it is necessary that:
$$J \alpha^2 - mgl > 0 \implies J \alpha^2 > mgl \implies \alpha^2 > \frac{mgl}{J}$$
If $J=ml^2$, then $\alpha^2 > \frac{mgl}{ml^2} = \frac{g}{l}$, i.e., $\alpha > \sqrt{\frac{g}{l}}$.

**Thus, to obtain two equal negative real eigenvalues $\lambda_0 = -\alpha$ (where $\alpha > \sqrt{g/l}$ for a positive $K_p$ when $J=ml^2$), the PD controller gains should be chosen as follows:**

-   **$K_d = 2 J \alpha$**
-   **$K_p = J \alpha^2 - mgl$**

**Choosing $\alpha$:**
The parameter $\alpha$ determines how quickly the transient response decays (the larger $\alpha$, the faster). Very large values of $\alpha$ will lead to large values of $K_p$ and $K_d$, which can cause saturation of the control signal or make the system sensitive to measurement noise (due to large $K_d$). The choice of $\alpha$ is a trade-off.
If $\alpha \le \sqrt{g/l}$ (when $J=ml^2$), then $K_p$ will be $\le 0$. The system can still be stable if $K_p > -mgl$ (i.e., $J \alpha^2 - mgl > -mgl \implies J \alpha^2 > 0$, which is true for $\alpha > 0$). However, a negative or zero $K_p$ means that the proportional component does not help or even hinders the return to equilibrium, which is atypical for stabilizing an inverted pendulum.

This concludes the theoretical analysis. 