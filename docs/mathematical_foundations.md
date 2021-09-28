# Mathematical Foundations & Algorithmic Derivations

This document provides rigorous mathematical formulations, system equations, and algorithmic derivations underpinning Beacon.

---

## 1. Lie Group $SO(3)$ and Lie Algebra $\mathfrak{so}(3)$

Rotational kinematics in 3D Euclidean space are parameterized using the Special Orthogonal Lie group $SO(3)$ and its associated Lie algebra $\mathfrak{so}(3)$.

### 1.1 Skew-Symmetric Operator (Hat and Vee)
For an angular velocity or rotation vector $\boldsymbol{\omega} = [\omega_1, \omega_2, \omega_3]^T \in \mathbb{R}^3$, the Lie algebra hat operator $[\cdot]_\times: \mathbb{R}^3 \to \mathfrak{so}(3)$ generates a $3 \times 3$ skew-symmetric matrix:

$$[\boldsymbol{\omega}]_\times = \begin{bmatrix} 0 & -\omega_3 & \omega_2 \\ \omega_3 & 0 & -\omega_1 \\ -\omega_2 & \omega_1 & 0 \end{bmatrix}$$

The inverse vee operator $(\cdot)^\vee: \mathfrak{so}(3) \to \mathbb{R}^3$ extracts the generating vector:

$$([\boldsymbol{\omega}]_\times)^\vee = \boldsymbol{\omega}$$

### 1.2 Exponential Map (Rodrigues Formula)
The matrix exponential maps an element of the Lie algebra $[\boldsymbol{\theta}]_\times \in \mathfrak{so}(3)$ to a rotation matrix $\mathbf{R} \in SO(3)$. Defining the rotation angle $\theta = \|\boldsymbol{\theta}\|_2$ and unit axis $\mathbf{u} = \boldsymbol{\theta} / \theta$:

$$\exp([\boldsymbol{\theta}]_\times) = \mathbf{I}_3 + \frac{\sin\theta}{\theta} [\boldsymbol{\theta}]_\times + \frac{1 - \cos\theta}{\theta^2} [\boldsymbol{\theta}]_\times^2$$

For small angles ($\theta < 10^{-6}$), the Taylor expansion prevents numerical cancellation:

$$\exp([\boldsymbol{\theta}]_\times) \approx \mathbf{I}_3 + [\boldsymbol{\theta}]_\times + \frac{1}{2} [\boldsymbol{\theta}]_\times^2$$

### 1.3 Logarithmic Map
Given $\mathbf{R} \in SO(3)$, the rotation angle is extracted via the trace:

$$\theta = \arccos\left(\frac{\text{tr}(\mathbf{R}) - 1}{2}\right)$$

The corresponding generator vector is:

$$\boldsymbol{\theta} = \log(\mathbf{R})^\vee = \frac{\theta}{2 \sin\theta} (\mathbf{R} - \mathbf{R}^T)^\vee$$

---

## 2. 15-State Error-State Extended Kalman Filter (ES-EKF)

Beacon utilizes an indirect Error-State Kalman Filter formulation (ES-EKF) where the high-rate IMU strapdown mechanization integrates the nominal state non-linearly, while low-frequency error states and covariances are propagated and corrected linearly.

### 2.1 State Representation
The complete state consists of the true state $\mathbf{x}$, the nominal state $\hat{\mathbf{x}}$, and the error state $\delta\mathbf{x}$:

$$\mathbf{x} = \hat{\mathbf{x}} \oplus \delta\mathbf{x}$$

The 15-dimensional error state vector is defined as:

$$\delta\mathbf{x} = \begin{bmatrix} \delta\mathbf{p} \\ \delta\mathbf{v} \\ \delta\boldsymbol{\theta} \\ \delta\mathbf{b}_g \\ \delta\mathbf{b}_a \end{bmatrix} \in \mathbb{R}^{15}$$

where:
- $\delta\mathbf{p} \in \mathbb{R}^3$: Position error in world frame (m)
- $\delta\mathbf{v} \in \mathbb{R}^3$: Velocity error in world frame (m/s)
- $\delta\boldsymbol{\theta} \in \mathbb{R}^3$: Small angle attitude error in world frame (rad)
- $\delta\mathbf{b}_g \in \mathbb{R}^3$: Gyroscope bias error in body frame (rad/s)
- $\delta\mathbf{b}_a \in \mathbb{R}^3$: Accelerometer bias error in body frame ($m/s^2$)

### 2.2 Error State Transition Jacobian
Linearizing the continuous kinematic error equations yields the continuous dynamics matrix $\mathbf{F}_c$:

$$\mathbf{F}_c = \begin{bmatrix} \mathbf{0}_3 & \mathbf{I}_3 & \mathbf{0}_3 & \mathbf{0}_3 & \mathbf{0}_3 \\ \mathbf{0}_3 & \mathbf{0}_3 & -[\mathbf{R}(\hat{\mathbf{q}})(\mathbf{a}_m - \hat{\mathbf{b}}_a)]_\times & \mathbf{0}_3 & -\mathbf{R}(\hat{\mathbf{q}}) \\ \mathbf{0}_3 & \mathbf{0}_3 & -[\boldsymbol{\omega}_m - \hat{\mathbf{b}}_g]_\times & -\mathbf{I}_3 & \mathbf{0}_3 \\ \mathbf{0}_3 & \mathbf{0}_3 & \mathbf{0}_3 & \mathbf{0}_3 & \mathbf{0}_3 \\ \mathbf{0}_3 & \mathbf{0}_3 & \mathbf{0}_3 & \mathbf{0}_3 & \mathbf{0}_3 \end{bmatrix}$$

Discretizing over integration time step $\Delta t$ via first-order Taylor approximation gives $\mathbf{F}_x = \mathbf{I}_{15} + \mathbf{F}_c \Delta t$:

$$\mathbf{F}_x = \begin{bmatrix} \mathbf{I}_3 & \mathbf{I}_3 \Delta t & \mathbf{0}_3 & \mathbf{0}_3 & \mathbf{0}_3 \\ \mathbf{0}_3 & \mathbf{I}_3 & -[\hat{\mathbf{a}}_w]_\times \Delta t & \mathbf{0}_3 & -\mathbf{R}(\hat{\mathbf{q}})\Delta t \\ \mathbf{0}_3 & \mathbf{0}_3 & \mathbf{I}_3 - [\hat{\boldsymbol{\omega}}_w \Delta t]_\times & -\mathbf{I}_3 \Delta t & \mathbf{0}_3 \\ \mathbf{0}_3 & \mathbf{0}_3 & \mathbf{0}_3 & \mathbf{I}_3 & \mathbf{0}_3 \\ \mathbf{0}_3 & \mathbf{0}_3 & \mathbf{0}_3 & \mathbf{0}_3 & \mathbf{I}_3 \end{bmatrix}$$

where $\hat{\mathbf{a}}_w = \mathbf{R}(\hat{\mathbf{q}})(\mathbf{a}_m - \hat{\mathbf{b}}_a)$.

### 2.3 Joseph Form Covariance Update
To guarantee numerical stability, positive semi-definiteness, and symmetry under floating-point roundoff, the posterior covariance $\mathbf{P}_+$ is updated using the stabilized Joseph formula:

$$\mathbf{P}_+ = (\mathbf{I} - \mathbf{K}\mathbf{H}) \mathbf{P}_- (\mathbf{I} - \mathbf{K}\mathbf{H})^T + \mathbf{K}\mathbf{R}_m\mathbf{K}^T$$

where the Kalman gain $\mathbf{K}$ is:

$$\mathbf{K} = \mathbf{P}_- \mathbf{H}^T (\mathbf{H} \mathbf{P}_- \mathbf{H}^T + \mathbf{R}_m)^{-1}$$

### 2.4 Error Injection and Reset
Following each measurement update, the estimated error state $\delta\hat{\mathbf{x}}$ is injected into the nominal state:

$$\hat{\mathbf{p}} \leftarrow \hat{\mathbf{p}} + \delta\hat{\mathbf{p}}$$
$$\hat{\mathbf{v}} \leftarrow \hat{\mathbf{v}} + \delta\hat{\mathbf{v}}$$
$$\hat{\mathbf{q}} \leftarrow \hat{\mathbf{q}} \otimes \exp\left(\frac{1}{2} [\delta\hat{\boldsymbol{\theta}}]_\times\right)$$
$$\hat{\mathbf{b}}_g \leftarrow \hat{\mathbf{b}}_g + \delta\hat{\mathbf{b}}_g$$
$$\hat{\mathbf{b}}_a \leftarrow \hat{\mathbf{b}}_a + \delta\hat{\mathbf{b}}_a$$

The error state vector is subsequently reset to zero: $\delta\mathbf{x} \leftarrow \mathbf{0}_{15}$.

---

## 3. Non-linear Model Predictive Control (LTV-MPC)

The Model Predictive Controller solves a constrained finite-horizon optimal control problem over horizon $N$:

$$\min_{\mathbf{u}_0, \dots, \mathbf{u}_{N-1}} J = \sum_{k=0}^{N-1} \left( \|\mathbf{z}_k - \mathbf{z}_k^{ref}\|_{\mathbf{Q}}^2 + \|\mathbf{u}_k\|_{\mathbf{R}}^2 + \|\Delta \mathbf{u}_k\|_{\mathbf{R}_d}^2 \right) + \|\mathbf{z}_N - \mathbf{z}_N^{ref}\|_{\mathbf{Q}_f}^2$$

subject to:
$$\mathbf{z}_{k+1} = \mathbf{f}(\mathbf{z}_k, \mathbf{u}_k)$$
$$a_{min} \le a_k \le a_{max}$$
$$-\delta_{max} \le \delta_k \le \delta_{max}$$
$$|\delta_{k} - \delta_{k-1}| \le \dot{\delta}_{max} \Delta t$$

### 3.1 Kinematic Bicycle Dynamics
The vehicle state is $\mathbf{z} = [x, y, \psi, v]^T$ and input is $\mathbf{u} = [a, \delta]^T$. The continuous kinematics are:

$$\dot{x} = v \cos(\psi + \beta)$$
$$\dot{y} = v \sin(\psi + \beta)$$
$$\dot{\psi} = \frac{v}{L} \cos(\beta) \tan(\delta)$$
$$\dot{v} = a$$

where side-slip angle $\beta = \arctan\left(\frac{l_r}{L} \tan(\delta)\right)$.

---

## 4. Quintic Hermite Splines ($C^2$ Continuity)

Quintic splines interpolate positions, velocities, and accelerations smoothly between waypoints with guaranteed $C^2$ continuous curvature.

The spline polynomial $p(t)$ for normalized parameter $t \in [0, 1]$ is:

$$p(t) = h_0(t) p_0 + h_1(t) v_0 + h_2(t) a_0 + h_3(t) a_1 + h_4(t) v_1 + h_5(t) p_1$$

The Hermite basis polynomials are:

$$h_0(t) = 1 - 10t^3 + 15t^4 - 6t^5$$
$$h_1(t) = t - 6t^3 + 8t^4 - 3t^5$$
$$h_2(t) = \frac{1}{2} t^2 - \frac{3}{2} t^3 + \frac{3}{2} t^4 - \frac{1}{2} t^5$$
$$h_3(t) = \frac{1}{2} t^3 - t^4 + \frac{1}{2} t^5$$
$$h_4(t) = -4t^3 + 7t^4 - 3t^5$$
$$h_5(t) = 10t^3 - 15t^4 + 6t^5$$

The signed planar curvature $\kappa(t)$ is computed analytically:

$$\kappa(t) = \frac{x'(t) y''(t) - y'(t) x''(t)}{\left(x'(t)^2 + y'(t)^2\right)^{3/2}}$$

---

## 5. Kuhn-Munkres (Hungarian Algorithm) $O(N^3)$

The Linear Sum Assignment Problem finds a bijective matching between $N$ agents and $N$ tasks minimizing total matching cost:

$$\min \sum_{i=1}^N \sum_{j=1}^N C_{ij} X_{ij}$$

subject to:
$$\sum_{j=1}^N X_{ij} = 1 \quad \forall i, \qquad \sum_{i=1}^N X_{ij} = 1 \quad \forall j, \qquad X_{ij} \in \{0, 1\}$$

Beacon implements the augmenting path formulation maintaining dual variables $u_i$ and $v_j$ such that:

$$u_i + v_j \le C_{ij} \quad \forall i, j$$

In each phase, an augmenting path is found in the equality subgraph ($u_i + v_j = C_{ij}$) using Dijkstra-like potential updates, guaranteeing optimal $O(N^3)$ complexity.
