## Task 2 (Trajectory Tracking) Objectives

- Generate a smooth joint-space trajectory between two joint configurations. 
- You can use either a trapezoidal velocity profile or a quintic polynomial trajectory. 
- Then track that trajectory and plot the desired joint positions versus the actual joint positions.

## Mathematics

First, the generalized velocity equations for the joint-space trajectory using a trapezoidal velocity profile were formed.

v(t) = at, 0 <= t < t1
v(t) = b, t1 <= t < t2
v(t) = -at + c, t2 <= t < t3

By integrating these equations with respect to time, you are left with the model for positon.

q(t) = at^2 / 2 + q(0), 0 <= t < t1
q(t) = bt + at1^2 / 2 + q(0) - bt1, t1 <= t < t2
q(t) = -at^2 / 2 + ct + bt2 + at1^2 / 2 + q(0) - bt1 + at2^2 / 2 - ct2

Currently there are two problems with my first attempt. Firstly, the final integrated equations are not very readable and will be hard to debug. Secondly, because I chose the acceleration constant, a, and the deceleration constant, a negated, to be equal, these equations are not generalized. 

I first wrote down some definitions.

area = (½·t₁·v_peak) + ((t₂−t₁)·v_peak) + (½·(t₃−t₂)·v_peak)
area = v_peak · [ t₁/2 + (t₂−t₁) + (t₃−t₂)/2 ]
where time_factor = [ t₁/2 + (t₂−t₁) + (t₃−t₂)/2 ]

time_factor · v_peak = target - init #total displacement
v_peak = (target - init) / time_factor

I then looked at my constants to solve this generalization problem
I noticed a is the acceleration constant:
accel = v_peak / t1 #rise/run of the velocity graph
And -a is the deceleration constant:
decel = v_peak / (t3 - t2) #rise/run of the velocity graph
This sucessesfully only uses the definitions provided and time parameters of t1-t3.


Looking at my first set of equations, much simplification could be performed with the initial value conditions such as position at t1 and t2.
pos_at_t1 = init + accel * t1**2 / 2 #inital position is a variable
pos_at_t2 = pos_at_t1 + v_peak * (t2 - t1)

q(t) = self.init + accel * t ** 2 / 2, 0 <= t < t1
q(t) = pos_at_t1 + self.v_peak * (t - self.t1), t1 <= t < t2
q(t) = pos_at_t2 + self.v_peak * dt - decel * dt ** 2 / 2 #dt = t - self.t2

## Initial Testing

Putting in the PID gains I found from Task 1, I graphed inital velocity trajectory graphs of each joint. The graph shows average error, max error, and I normalized the average error against each joint's peak target velocity to calculate an alignment percentage (100% meaning average error of zero relative to that joint's velocity range). Adding a position vs time graph was also a helpful visualization of how the robot is moving.

![no feedforward](task2.2_images/no_feedforward.png)

The first test showed relatively poor alignment to the velocity graph with a overall alignment of 98.4%.

## Feedforward

Pure feedback control is reactive: it only produces torque once a tracking error already exists. When tracking a moving setpoint like the trapezoidal velocity profile, this means the loop has to build up a persistent position error just to keep producing the torque needed to track a non-zero target velocity, which is exactly the lag/error seen in Initial Testing.

The fix is setpoint/command feedforward, a standard 2-DOF control architecture: since the target velocity is known analytically ahead of time (from `compute_tgt_vel`), it can be fed directly into the control signal in parallel with feedback, instead of relying on tracking error to reveal it after the fact. Feedback (`Kp`, `Ki`, and `- Kd*(curr_qvel)`) then only needs to correct for residual error and model mismatch, while feedforward (`Kv * target_qvel`) supplies the bulk of the torque the trajectory already tells us is needed.

$$u(t) = K_p e(t) + K_i \int_{0}^{t} e(\tau) d\tau + K_d \frac{de(t)}{dt} + K_v \dot{q}_{target}(t)$$

https://apmonitor.com/pdc/index.php/Main/FeedforwardControl
https://web.stanford.edu/class/archive/ee/ee392m/ee392m.1034/Lecture5_Feedfrwrd.pdf











## Links

https://www.youtube.com/watch?v=-oGNxB86YEk