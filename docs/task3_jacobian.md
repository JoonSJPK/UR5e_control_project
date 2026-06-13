## Task 1 (Joint Control) Objectives

- Using inverse kinematics or a Jacobian-based resolved-rate control approach, move the end-effector toward the CAD object you created. 
- If possible, also visualize the Cartesian path of the end-effector. 

## Step 1 (Forward Kinematics)

### Denavit-Hartenberg Convention
A robot arm consists of rigid links that are connected by joints. In forward kinematics, a coordinate frame is placed onto each link (x, y, z axis). 
The process of foward kinematics allows you to find the cartesian coordinates of a end effector of the UR5e arm based off of joint angles (theta) and link lengths (m). 

In general, relating one frame to another in 3D takes 6 numbers (3 position, 3 orientation). However, the Denavit-Hartenberg convention aligns each frame's z-aix with its joint's rotation axis and its x axis along the common perpendicular to the next joint, therefore, allowing us to use only 4 numbers to describe how to travel from frame i - 1 to frame i.

θi​ — rotate about z. For a revolute joint this is the joint angle: what the motor turns.
did_i
di​ — slide along z (the link offset, how far the next joint is stacked along this axis).
aia_i
ai​ — slide along x (the link length, how far the next joint reaches sideways).
αi\alpha_i
αi​ — rotate about x (the link twist, how much the next joint's axis is tilted relative to this one).

This information what gathered from a UR5e datasheet and organize in a table. From this table, the only changing variable is the joint position(theta). d, a and alpha are constants.

DH parameter table

| Joint i | θᵢ | dᵢ | aᵢ | αᵢ |
|---------|------|--------|---------|------|
| 1 | θ0 | 0.1625 | 0 | 90° |
| 2 | θ1 | 0 | −0.425 | 0° |
| 3 | θ2 | 0 | −0.3922 | 0° |
| 4 | θ3 | 0.1333 | 0 | 90° |
| 5 | θ4 | 0.0997 | 0 | −90° |
| 6 | θ5 | 0.0996 | 0 | 0° |

https://www.universal-robots.com/articles/ur/application-installation/dh-parameters-for-calculations-of-kinematics-and-dynamics/


