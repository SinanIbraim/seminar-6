# Taking a look back at the pendulum

$$\dot{s}_1 = s_2$$

$$\dot{s}_2 = -\frac{g}{l}\sin s_1 + \frac{a}{ml^2}$$ *settling time*

## Actual control:
current $i$

## Motor dynamics:

$$\dot{\tau} = \frac{1}{\gamma}(-\tau + a)$$ *torque command*

## So, overall plant reads now:

$$\dot{s}_1 = s_2$$

$$\dot{s}_2 = -\frac{g}{l}\sin s_1 + \frac{s_3}{ml^2}$$

$$\dot{s}_3 = \frac{1}{\gamma}(-s_3 + a)$$

$$s = \begin{pmatrix} \theta \\ \dot{\theta} \\ \tau \end{pmatrix}$$

We consider $(s_1, s_2)$ -subplant with $s_3$ being interpreted as action(!)

We know there is a policy $\pi_0$, say, energy-based upswing (+ possibly a local PID etc.)

And we know its "certificate", i.e., a LF $L_0$ s.t.

$$\dot{L}_0(s|\pi_0) < 0$$

## Question:
can we exploit $(\pi_0, L_0)$ to design a controller for the extended plant? I.e., $(s_1, s_2, s_3)$-plant

## Answer:
backstepping!

Backstepping suggests a LF candidate via augmentation of $L_0$ (cf. adaptive control)

$$L_1 := L_0 + \frac{1}{2}||s_3 - \pi_0(s_1, s_2)||^2$$

This assumes (conveniently) a plant structure as:

$$\dot{s} = f(s) + g(s)c$$
$$\dot{c} = a$$

Let's compare
| Want: | Have: |
|-------|-------|
| $$\dot{s} = f(s) + g(s)c$$ | $$\dot{s}_1 = s_2$$ |
| $$\dot{c} = a'$$ | $$\dot{s}_2 = -\frac{g}{l}\sin s_1 + \frac{s_3}{ml^2}$$ |
|  | $$\dot{s}_3 = \frac{1}{\gamma}(-s_3 + a)$$ |

## Coordinate transformation:

$$c := s_3$$

$$a' := \frac{1}{\gamma}(-c + a)$$

$$\Rightarrow$$

$$\dot{s} = f(s) + g(s)c$$
$$\dot{c} = a'$$

## More general case:

$$\dot{s} = f(s) + g(s)c$$
$$\dot{c} = f_1(s, c) + g(s, c)a$$

$$\downarrow?$$

$$\dot{s} = f(s) + g(s)c'$$
$$\dot{c}' = a'$$

Then, do a coord. transform to get

$$a' := f_1(s,c) + g(s,c)a$$
$$c' := c$$

Evidently, this won't work if the "old" subplant is coupled on the "new", i.e. if $f, g$ depend on $c$

Let's get back to backstepping.

From now on, we work in the following context:

$$\dot{s} = f(s) + g(s)c$$
$$\dot{c} = a$$

There is $\pi_0(s)$ which, when substituted in place of $c$, gives a closed-loop LF $L_0$ for the $s$-subplant.

Backstepping: $$L_1 := L_0 + \frac{1}{2}||c - \pi_0(s)||^2$$

Inspect:

$$\dot{L}_1 = \underbrace{L_f L_0}_{\text{}} + \underbrace{L_g L_0 c}_{\text{}} + (c - \pi_0(s))(a - \underbrace{L_f\pi_0 - L_g\pi_0 c}_{\text{}})$$

$$\leftarrow \text{ Notice difference}$$

We know that $$L_f L_0 + L_g L_0 \pi_0(s) < 0$$

Suggestion:

$$a := -k(c - \pi_0(s))$$
$$\quad \uparrow \text{ Control gain}$$

Then,

$$\dot{L}_1 = \underbrace{L_f L_0}_{\text{}} + \underbrace{L_g L_0 c}_{\text{}} - k||c-\pi_0(s)||^2 - (c-\pi_0(s))(\underbrace{L_f\pi_0 + L_g\pi_0 c}_{\text{}})$$

We want to recover

$$\underbrace{L_f L_0}_{\text{}} + \underbrace{L_g L_0 \pi_0(s)}_{\text{}} \text{ ! (see below)}$$

So, modify the policy as follows:

$$a := -k ||c-\pi_0(s)||^2 - L_g L_0 + L_{f+gc} \pi_0$$

Then, the effect of $-L_g L_0$ is:

$$-(c-\pi_0(s))^T L_g L_0 = -L_g L_0 c + \underbrace{L_g L_0 \pi_0(s)}_{\text{}} \text{ ! (see above)}$$

Hence, $$\dot{L}_1 \leq -K_{dec,0}(||s||) - k||c-\pi_0(s)||^2$$

## Another example:

$$\dot{x} = v\cos\theta$$
$$\dot{y} = v\sin\theta$$
$$\dot{\theta} = \omega$$

(diff. drive robot)

$$\dot{v} = \frac{1}{m}f$$
$$\dot{\omega} = \frac{1}{J}\tau$$

This at once (except for coefficients $k_m$, $k_J$) in the form we worked with