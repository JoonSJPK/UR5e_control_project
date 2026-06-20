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









## Links

https://www.youtube.com/watch?v=-oGNxB86YEk