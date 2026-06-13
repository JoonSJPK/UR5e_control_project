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

### Elementary Transformation
Each of the four parameters corresponds to one elementary transform which is a basic rotation or translation described as a 4×4 matrix.

Transformation equation
Ti−1i​=Rotz​(θ)Transz​(d)Transx​(a)Rotx​(α)

Cθ = cos θ, Sθ = sin θ, Cα = cos α, Sα = sin α.

The transformation equation

$$T_{i-1}^{\,i} = \text{Rot}_z(\theta)\;\text{Trans}_z(d)\;\text{Trans}_x(a)\;\text{Rot}_x(\alpha)$$

The four matrices

**Rotₓ z(θ)**: rotation about z by the joint angle:

$$\text{Rot}_z(\theta)=\begin{bmatrix} C\theta & -S\theta & 0 & 0\\ S\theta & C\theta & 0 & 0\\ 0 & 0 & 1 & 0\\ 0 & 0 & 0 & 1 \end{bmatrix}$$

**Transz(d)**: translation along z by the link offset:

$$\text{Trans}_z(d)=\begin{bmatrix} 1 & 0 & 0 & 0\\ 0 & 1 & 0 & 0\\ 0 & 0 & 1 & d\\ 0 & 0 & 0 & 1 \end{bmatrix}$$

**Transₓ(a)**: translation along x by the link length:

$$\text{Trans}_x(a)=\begin{bmatrix} 1 & 0 & 0 & a\\ 0 & 1 & 0 & 0\\ 0 & 0 & 1 & 0\\ 0 & 0 & 0 & 1 \end{bmatrix}$$

**Rotₓ(α)**: rotatation about x by the link twist:

$$\text{Rot}_x(\alpha)=\begin{bmatrix} 1 & 0 & 0 & 0\\ 0 & C\alpha & -S\alpha & 0\\ 0 & S\alpha & C\alpha & 0\\ 0 & 0 & 0 & 1 \end{bmatrix}$$

The combined result

$$T_{i-1}^{\,i}=\begin{bmatrix} C\theta & -S\theta\,C\alpha & S\theta\,S\alpha & a\,C\theta\\ S\theta & C\theta\,C\alpha & -C\theta\,S\alpha & a\,S\theta\\ 0 & S\alpha & C\alpha & d\\ 0 & 0 & 0 & 1 \end{bmatrix}$$

### Chaining Transformations Together For Forward Kinematics

$$T_0^{\,1} = \begin{bmatrix} \cdots \end{bmatrix} \qquad T_1^{\,2} = \begin{bmatrix} \cdots \end{bmatrix}$$

**Cumulative products:**

$$T_0^{\,2} = T_0^{\,1}\,T_1^{\,2}$$
$$T_0^{\,3} = T_0^{\,2}\,T_2^{\,3}$$
$$\vdots$$
$$T_0^{\,6} = T_0^{\,5}\,T_5^{\,6}$$

$T_0^{\,1}$: "the transform from frame 0 to frame 1"

**Definitions:**

$T_{i-1}^{\,i}$ = an elementary step, one joint to the next

$T_0^{\,i}$ = a cumulative transform

$$T_0^{\,6} = T_0^{\,1}\,T_1^{\,2}\,T_2^{\,3}\,T_3^{\,4}\,T_4^{\,5}\,T_5^{\,6}$$

**Structure of the cumulative transform:**

$$T_0^{\,i} = \left[\begin{array}{ccc|c} & & & \\ & R_0^{\,i} & & P_i \\ & & & \\ \hline 0 & 0 & 0 & 1 \end{array}\right]$$

- Column 1 → frame *i*'s x-axis
- Column 2 → frame *i*'s y-axis
- Column 3 → frame *i*'s z-axis = $z_i$
- Column 4 → the origin $p_i$

The top right column shows pe, it is the end effector position in Cartesian coordinates measured in meters. The top left 3x3 matrix Re describes the eneffectors orientation.



