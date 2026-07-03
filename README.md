# UR5e Control Project

Simulated control of a Universal Robots UR5e arm in MuJoCo. This repository covers three tasks: a custom torque-controlled PID joint controller, joint-space trajectory tracking with feedforward, and Jacobian-based inverse kinematics that drives the end-effector to a target pose and grasps a bottle.

Each task has a full writeup in `docs/` that explains the math and the tuning process:

- [Task 1: Joint Control](docs/task1_joint_control.md)
- [Task 2: Trajectory Tracking](docs/task2_trajectory_tracking.md)
- [Task 3: Jacobian](docs/task3_jacobian.md)

## Setup

The project targets Python 3 with MuJoCo 3.9. Create a virtual environment and install the dependencies.

```
python3 -m venv .venv
source .venv/bin/activate
pip install -r requirements.txt
```

The UR5e model comes from MuJoCo Menagerie and lives in `models/universal_robots_ur5e/`. The scene (`scene.xml`) already includes the Fusion360 bottle and, for Task 3, the Franka Panda gripper.

On macOS the MuJoCo interactive viewer must be launched with `mjpython` instead of `python`. `mjpython` is installed alongside MuJoCo, so it sits in the same directory as the `python` in the virtual environment. Commands below that open a viewer use `mjpython`.

## Running the tasks

All commands are run from the `src/` directory.

```
cd src
```

### Task 1: Joint Control

Task 1 uses an ITAE gain sweep and a PID controller applied as direct joint torques. The main interface is a Tkinter GUI that lets you set per-joint gains and targets, run the simulation, and plot the tracking error.

```
python -m task1_joint_control.gui
```

The GUI spawns the simulation through `mjpython` on its own. To run a single configuration headlessly and save the plot and data instead, call the controller directly. The `--joints` argument is a comma-separated list of `joint:kp:ki:kd:target[:integral_limit_nm]`.

```
mjpython -m task1_joint_control.joint_control \
  --joints "1:100:10:5:-1.5708" \
  --out plot.png --data-out data.csv
```

The ITAE sweep that produced the gains can be run on its own with `python -m task1_joint_control.itea_sweep`.

### Task 2: Trajectory Tracking

Task 2 generates a trapezoidal velocity profile between two joint configurations and tracks it with PID plus command feedforward. The initial and target configurations are set at the top of `main()` in `trajectories.py`.

```
mjpython task2_trajectory_tracking/trajectories.py
```

This opens the viewer and saves the desired-versus-actual velocity and position plots.

### Task 3: Jacobian

Task 3 runs a resolved-rate inverse kinematics loop that drives the end-effector to the bottle, closes the gripper, and returns to the home pose. Forward kinematics is validated against MuJoCo at startup.

```
mjpython task3_jacobian/jacobian.py
```

## Main results

### Task 1

Rather than tuning the four coupled parameters (Kp, Ki, Kd, and the integral limit) by hand, I used an ITAE sweep extended with a torque-overage penalty weighted by λ. A value of λ = 1e-8 was the balance point where the joints stay just under their torque limits, barely saturate, and keep steady-state error low without becoming sluggish. The remaining steady-state error came from uncompensated gravity, which the `qfrc_bias` term resolved cleanly and made the integral gain unnecessary; however, `qfrc_bias` was left off for the later tasks to keep exploring PID tuning on its own.

### Task 2

Pure PID feedback produced a reactive lag with an overall velocity alignment of 98.4%, since the loop needed a buildup of error before it could supply torque for a moving target. Adding command feedforward (`Kv * target_qvel`) closed most of that gap to 99.6% by supplying the expected torque ahead of time. Joint 2 was the weakest performer because it carries the rest of the arm against gravity. Raising its Kp and Ki specifically brought it in line with the other joints.

### Task 3

The Denavit-Hartenberg forward kinematics agrees with MuJoCo to within 2 mm across the test poses. A damped least-squares solve drives the end-effector using both a 3x6 position-only Jacobian and the full 6x6 Jacobian for position and orientation. With the Franka Panda gripper mounted, the arm reaches the bottle, closes, and returns while holding it. Reliable grasping required gripping near the center of mass, raising the fingertip friction, and stiffening the gripper servo so the contact normal force was enough to resist slipping.
