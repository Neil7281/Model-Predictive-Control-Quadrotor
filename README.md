# Model-Predictive-Control-Quadrotor
📌 Overview
This project presents a Proportional-Derivative (PD) control framework optimized for accurate trajectory tracking of a quadrotor, especially under the influence of aerodynamic disturbances. The work involves dynamic modeling, PD controller implementation, and gradient-based optimization of control parameters to enhance tracking accuracy across various trajectories.

🎯 Objectives
Develop a nonlinear dynamic model of a quadrotor based on literature.

Implement a PD controller for trajectory tracking.

Optimize the control parameters using gradient-based optimization techniques.

Evaluate performance on different reference trajectories: Circular, Helical, and Lissajous.

⚙️ Methodology
1. Dynamic Modeling
A full nonlinear model of the quadrotor is derived considering its 6-DOF motion and aerodynamic influences.

2. PD Control Strategy
A PD controller is implemented to regulate quadrotor position and attitude using the error between the desired and actual states.

3. Parameter Optimization
Control gains are optimized using a gradient-based approach to minimize the integral of squared error (ISE) over each trajectory.

📈 Results
Trajectory tracking performance was evaluated for:

Circular Trajectory: Demonstrated minimal error with optimized gains.

Helical Trajectory: Controller adapted well to 3D path variations.

Lissajous Trajectory: Showed sensitivity to gain tuning, emphasizing the need for trajectory-specific parameters.

✅ Conclusion
The optimized PD controller significantly improved trajectory tracking accuracy.

Gain values differ per trajectory, validating the importance of trajectory-specific tuning.

The study provides a base for adaptive control strategies in future research.

🔭 Future Work
The next step is to design an adaptive controller that dynamically adjusts PD gains in real-time for any given trajectory, ensuring robust performance in dynamic environments. Applications include:

Precision agriculture

Autonomous surveillance

Aerial delivery systems

📚 References
Garima Bhandari, et al., 2022. Bond graph modeling and trajectory control of H-drone, 13th Asian Control Conference.

T.P. Nascimento & M. Saska, 2019. Position and attitude control of multi-rotor aerial vehicles: A survey, Annual Reviews in Control.

M.K. Sahoo, J.K. Dutt, S.K. Saha, 2021. Quadcopter control using the viscoelastic control law, ICCAS.

👤 Author
Neelkumar Subhashbhai Ahir
Supervised by Prof. Garima Bhandari

