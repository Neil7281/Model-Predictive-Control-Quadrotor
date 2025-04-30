# Optimized PD Controller for Quadrotor Trajectory Tracking
This repository presents a simulation-based control framework for trajectory tracking of a quadrotor under aerodynamic influences. Initially focused on Proportional-Derivative (PD) control, the project implements gradient-based optimization for fine-tuning controller gains and sets the foundation for future research in robust control using Model Predictive Control (MPC).

🎯 Objectives
Model the full nonlinear dynamics of a quadrotor.

Design and implement a PD control strategy for trajectory tracking.

Optimize PD control gains using gradient-based optimization to minimize trajectory tracking error.

Analyze controller performance on multiple reference trajectories: Circular, Helical, and Lissajous.

Lay the groundwork for future implementation of MPC-based control.

⚙️ Methodology
1. Dynamic Modeling
A nonlinear 6-DOF model of a quadrotor is developed, incorporating translational and rotational dynamics under external disturbances.

2. PD Control Architecture
A traditional PD controller is applied to regulate position and orientation. Controller performance depends on the tuning of Kp and Kd gains.

3. Optimization of Control Gains
An objective function based on the Integral of Squared Error (ISE) is minimized using gradient-based optimization to identify optimal control parameters for each trajectory.

📈 Simulation Results
Tracking performance was validated on three complex trajectories:

Circular Trajectory – Demonstrated consistent convergence to reference path.

Helical Trajectory – Showed stability and smooth elevation tracking.

Lissajous Trajectory – Captured dynamic path changes with reduced overshoot.

✅ Conclusion
The optimized PD controller successfully enhanced the quadrotor’s tracking accuracy across diverse trajectory types.

Gain values significantly varied with the trajectory, highlighting the importance of task-specific tuning.

The current PD-based solution provides a strong baseline for robust trajectory tracking.

🔮 Next Steps: Model Predictive Control (MPC)
To overcome the limitations of fixed-gain PD controllers, future work will implement Model Predictive Control:

MPC can handle multi-variable systems and constraints in real time.

It allows predictive optimization over a moving horizon, enabling the system to adapt to changing dynamics and external disturbances.

This approach is expected to offer robustness, adaptability, and optimal performance in uncertain and complex environments.


📚 References
Bhandari, G. et al., 2022 – Bond graph modeling and trajectory control of H-drone (Asian Control Conference)

Nascimento, T.P. & Saska, M., 2019 – Position and attitude control of multi-rotor aerial vehicles: A survey (Annual Reviews in Control)

Sahoo, M.K. et al., 2021 – Quadcopter control using viscoelastic control law (ICCAS)

👤 Author
Neelkumar Subhashbhai Ahir
Supervisor: Prof. Garima Bhandari


