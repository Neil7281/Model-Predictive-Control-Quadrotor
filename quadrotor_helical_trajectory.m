
function quadrotor_6DOF_control()
global w1_des_history w2_des_history w3_des_history w4_des_history t_history phi_ref_history theta_ref_history phi_history;
    w1_des_history = [];
    w2_des_history = [];
    w3_des_history = [];
    w4_des_history = [];
    phi_ref_history = [];
    theta_ref_history = [];
    t_history = [];
    % Simulation parameters
    tspan = [0:0.01:150]; % Time span for simulation (seconds)
    
    % Initial conditions: [x, y, z, x_dot, y_dot, z_dot, phi, theta, psi, phi_dot, theta_dot, psi_dot]
    initial_conditions = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0];

    % Solve the differential equations using ode45
    options = odeset('RelTol',1e-3,'AbsTol',1e-16);
    [t, states] = ode45(@quadrotor_dynamics, tspan, initial_conditions, options);

   % [t, states] = ode45(@quadrotor_dynamics, tspan, initial_conditions);

    %zdesired = 0;xref = 0; yref =1;
    %zdesired = 10;xref = 3*cos(0.1*t); 
    %yref = 3*sin(0.1*t);

    % Extract the position and angles from the state variables
    x = states(:,1); y = states(:, 2); z = states(:, 3);z_dot = states(:, 6);
    phi = states(:, 7); theta = states(:, 8); psi = states(:, 9);phi_ref = states(:,13);theta_ref = states(:,14);
    %xref = states(:,15); yref = states(:,16);
    zdesired = 10*ones(size(t));

    % Plot the position over time
    xref = 3*cos(0.1*t);
    yref = 3*sin(0.1*t);
    zdesired = 2*t;
    psi_ref = zeros(size(t));
    plot3(x,y,z, '--','LineWidth', 6);%xlabel('X'); ylabel('Y'); zlabel('z');title('Positions');
    hold on 
    %plot3(xref,yref,zdesired,'LineWidth', 2);
    hold off
    figure;
    %subplot(1, 1, 1); plot3(x,y,z); xlabel('Time (s)'); ylabel('x Position (m)');zlabel('x Position (m)'); title('x Position');
   
   
  
    subplot(3, 1, 1); plot(t,x,'LineWidth', 4); xlabel('Time (s)'); ylabel('X (m)'); title('X Position');
    hold on 
    plot(t, xref,"--",'LineWidth', 3);
    hold off
    subplot(3, 1, 2); plot(t, y,'LineWidth', 3); xlabel('Time (s)'); ylabel('Y  (m)'); title('Y Position');
    hold on 
    plot(t,yref,"--",'LineWidth', 3);
    hold off
    subplot(3, 1, 3); plot(t, z,'LineWidth', 3); xlabel('Time (s)'); ylabel('Z (m)'); title('Z Position');%ylim([0 12]);
    hold on 
    plot(t,zdesired.*ones(size(t)),"--",'LineWidth', 4);
    hold off
    
    % Plot the Euler angles over time
    figure;
    plot(t,phi_ref); xlabel('Time (s)'); ylabel('Roll \phi '); title('Roll and Pitch Angle');
    hold on 
    plot(t,theta_ref);legend("phi ref", "theta ref");
    hold off
    figure;
    subplot(2,1,1);plot(t,phi_ref - phi);xlabel('Time (s)'); ylabel('error (m)'); title('phi error');
    subplot(2,1,2);plot(t,theta_ref - theta);xlabel('Time (s)'); ylabel('error (m)'); title('theta error');
    

    figure;
    subplot(3, 1, 1); plot(t, phi); xlabel('Time (s)'); ylabel('Roll \phi'); title('Roll Angle');ylim([-0.2 0.05]);
    hold on
    subplot(3, 1, 1); plot(t_history, phi_ref_history,"--"); xlabel('Time (s)'); ylabel('Roll \phi'); title('Roll Angle');
    hold off
    subplot(3, 1, 2); plot(t, theta); xlabel('Time (s)'); ylabel('Pitch \theta (deg)'); title('Pitch Angle');
    hold on 
    subplot(3, 1, 2); plot(t_history, theta_ref_history,"--"); xlabel('Time (s)'); ylabel('Pitch \theta'); title('Pitch Angle');
    hold off
    subplot(3, 1, 3); plot(t, psi); xlabel('Time (s)'); ylabel('Yaw \psi'); title('Yaw Angle');
    hold on
    subplot(3, 1, 3); plot(t, psi_ref,"--"); xlabel('Time (s)'); ylabel('Yaw \psi'); title('Yaw Angle');
    hold off
   % MATLAB Code to Calculate Integral Square Error (ISE)

% Step 1: Define the time vector and error vector
% Example: Simulate a control system response % Time vector from 0 to 10 seconds with 0.01s steps
error = (xref - x).^2 + (yref - y).^2 + (zdesired - z).^2; % Example error signal (replace with actual error data)
time_absolute_error = t.*abs(error);
% Step 2: Calculate the squared error
squared_error = error; % Element-wise square of the error

% Step 3: Integrate the squared error over time using the trapezoidal rule
ISE = trapz(t, squared_error); % Integral of squared_error with respect to t
ITAE = trapz(t, time_absolute_error);
% Step 4: Display the result
fprintf('Integral Square Error (ISE): %.4f\n', ISE);
fprintf('IntegraI Time Absolute Error (ITAE): %.4f\n', ITAE);
figure;
  plot(t_history, w1_des_history, 'LineWidth', 2);
  hold on 
  plot(t_history, w2_des_history, 'LineWidth', 2);
  plot(t_history, w3_des_history, 'LineWidth', 2);
  plot(t_history, w4_des_history, 'LineWidth', 2);
  hold off
  xlabel('Time (s)');
  ylabel('w1,w2,w3,w4(rad/s)');
  title('Motor Speed vs Time');
  legend('w1','w2','w3','w4');
  grid on;
% Step 5: Plot the error and squared error (optional)
figure;
subplot(3,1,1);
plot(t, error, 'b');
xlabel('Time (s)');
ylabel('Error');
title('Error vs Time');
grid on;


subplot(3,1,2);
plot(t, squared_error, 'r');
xlabel('Time (s)');
ylabel('ISE');
title('Integral Squared Error(ISE) vs Time');
grid on;

subplot(3,1,3);
plot(t, time_absolute_error, 'g');
xlabel('Time (s)');
ylabel('ITAE');
title('IntegraI Time Absolute Error (ITAE) vs Time');
grid on;
end


function dXdt = quadrotor_dynamics(t, X)
global w1_des_history w2_des_history w3_des_history w4_des_history t_history phi_ref_history theta_ref_history;
    % Quadrotor Parameters
    %m = 2.1;   % Mass of the quadrotor (kg)
    %g = 9.81;  % Gravitational acceleration (m/s^2)
    %Ixx = 0.107; Iyy = 0.2; Izz = 0.226; % Moment of inertia (kg*m^2)

  g = 9.81;            
m = 1.587;           
Ixx = 0.0213; Iyy = 0.02217; Izz = 0.0282; 
    k_f = 2.98*10^-6; k_m = 1.14*10^-7;
    l1 = 0.35;l2 = 0.56;
    kp_m = 300; ki_m = 5;
    
    % Extract state variables (corrected indexing)
     x = X(1); y = X(2); z = X(3); 
    x_dot = X(4); y_dot = X(5); z_dot = X(6);
    phi = X(7); theta = X(8); psi = X(9); 
    r_dot = X(10); p_dot = X(11); s_dot = X(12);
   % xref = X(15); yref = X(16);
    
    % Desired States
    zdesired = 2*t; xref = 3*cos(0.1*t);
    yref = 3*sin(0.1*t);
    %phi_ref = 0; theta_ref = 0; 
    psi_ref = 0;
    xref_dot =  -0.3*sin(0.1*t);
    yref_dot = 0.3*cos(0.1*t);
    xref_ddot = -0.03*cos(0.1*t);                                          % 
    yref_ddot = -0.03*sin(0.1*t);
    xref_dddot = 0.003*sin(0.1*t);
    yref_dddot = -0.003*cos(0.1*t);
    zdesired_dot = 2;
    
   % Control Gains for PID
k_p_pos_z = 163.0297; k_p_ang_phi = 380.8515;
k_d_pos_z = 110.0335; k_d_ang_phi = 150.4867;
k_p_pos_y = 0.1000; k_p_ang_theta = 400.4488;
k_d_pos_y = 0.16000; k_d_ang_theta = 100.0240;
k_p_pos_x = 0.09000; k_p_ang_psi = 100.1353;
k_d_pos_x = 0.1200; k_d_ang_psi = 280.7893;
k_i = 250;

    xddot = 0;
    yddot =0;
    xe = k_p_pos_x*(xref - x) + k_d_pos_x*(xref_dot - x_dot);
    ye = k_p_pos_y*(yref - y) + k_d_pos_y*(yref_dot - y_dot);
    xe_dot =  k_p_pos_x*(xref_dot - x_dot) + k_d_pos_x*(xref_ddot - xddot);
    ye_dot = k_p_pos_y*(yref_dot - y_dot) + k_d_pos_y*(yref_ddot - yddot);

    phi_ref = (xe*sin(psi)  - ye*cos(psi));
    theta_ref = (xe*cos(psi) + ye*sin(psi));
    phi_ref_history = [phi_ref_history; phi_ref];
    theta_ref_history = [theta_ref_history; theta_ref];
    
    phi_ref_dot = (xe_dot*sin(psi)  - ye_dot*cos(psi));
    theta_ref_dot = (xe_dot*cos(psi) + ye_dot*sin(psi));
    psi_ref_dot =0;
  
   

   P = [1 , 0, -sin(theta); 0, cos(phi), cos(theta)*sin(phi); 0, -sin(phi), cos(theta)*cos(phi)];
   I = P*[phi_ref_dot; theta_ref_dot; psi_ref_dot];
   r_ref_dot = I(1,:);
   p_ref_dot = I(2,:);
   s_ref_dot = I(3,:);
    
    % Hovering motor speed
    w_lift = k_p_pos_z*(zdesired - z) + k_d_pos_z*(zdesired_dot - z_dot);
    w_phi = k_p_ang_phi*(phi_ref - phi) + k_d_ang_phi*(r_ref_dot - r_dot);
    w_theta = k_p_ang_theta*(theta_ref - theta) + k_d_ang_theta*(p_ref_dot - p_dot);
    w_psi = k_p_ang_psi*(psi_ref - psi) + k_d_ang_psi*(s_ref_dot - s_dot);
    w_h = sqrt((m*g)/(4*k_f));
    % Rotor speeds calculation
    w = [0;0;0;0];
    %max_motor_speed = 1500; % Set a realistic maximum motor speed
    %min_motor_speed = 0;    % Set a minimum motor speed (usually zero)

    w1_des =  w_h + w_lift - w_phi + w_theta - w_psi;                       % [1 -1 1 -1]*[w_h + w_lift;w_phi;w_theta;w_psi];
    w2_des =  w_h + w_lift + w_phi + w_theta + w_psi;                        %[1 1 1 1]*[w_h + w_lift;w_phi;w_theta;w_psi];
    w3_des =  w_h + w_lift + w_phi - w_theta - w_psi;                        %[1 1 -1 -1]*[w_h + w_lift;w_phi;w_theta;w_psi];
    w4_des =  w_h + w_lift - w_phi - w_theta + w_psi;                        %[1 -1 -1 1]*[w_h + w_lift;w_phi;w_theta;w_psi];
    w_des = [w1_des; w2_des; w3_des; w4_des];
      w_des = [w1_des; w2_des; w3_des; w4_des];
    w1_des_history = [w1_des_history; w1_des];
    w2_des_history = [w2_des_history; w2_des];
    w3_des_history = [w3_des_history; w3_des];
    w4_des_history = [w4_des_history; w4_des];
    t_history = [t_history; t];
   
    % Forces 
    F1  = k_f*w1_des^2;
    F2  = k_f*w2_des^2;
    F3  = k_f*w3_des^2;
    F4  = k_f*w4_des^2;
    F_z = F1 + F2 + F3 + F4;
   % moments
    
    M1  = k_m*w1_des^2;
    M2  = k_m*w2_des^2;
    M3  = k_m*w3_des^2;
    M4  = k_m*w4_des^2;


    % Torques
    T_x = ((F3 + F2) - (F1 + F4))*l1;   % Roll motion
    T_y = ((F1 + F2) - (F3 + F4))*l2;   % Pitch motion
    T_z = (M2 + M4 - M1 - M3); % Yaw motion


   T = [T_x;T_y;T_z];

   R  = [cos(theta)*cos(psi)   sin(phi)*cos(theta)*cos(psi)-cos(phi)*sin(psi)    cos(phi)*sin(theta)*cos(psi)-sin(theta)*sin(psi); cos(theta)*sin(psi)   sin(phi)*sin(theta)*sin(psi)-cos(phi)*cos(psi)    cos(phi)*sin(theta)*sin(psi)-sin(phi)*cos(psi);-sin(theta)   sin(phi)*cos(theta)   cos(phi)*cos(theta)];
   
   %[r_dot;p_dot;s_dot] = R.*[phi_dot; theta_dot; psi_dot];
   P1 = [1,sin(phi)*tan(theta),cos(phi)*tan(theta); 0,cos(phi),-sin(phi); 0,-sin(phi)/cos(theta),cos(phi)/cos(theta)];
   %P = [1,0,cos(theta);0,cos(phi),sin(phi)*cos(theta);0,-sin(phi),cos(phi)*cos(theta)];
   P = [1 , 0, -sin(theta); 0, cos(phi), cos(theta)*sin(phi); 0, -sin(phi), cos(theta)*cos(phi)];
   %I = P*[phi_dot; theta_dot; psi_dot];
   %r_dot = I(1,:);
   %p_dot = I(2,:);
   %s_dot = I(3,:);

   x_ddot = (s_dot*y_dot - p_dot*z_dot + g*sin(theta)) - (0.25*x_dot)/m;                                                            
   y_ddot = r_dot*z_dot - s_dot*x_dot - g*sin(phi)*cos(theta) - (0.25*y_dot)/m;                                                   
   z_ddot = (F_z/m) + p_dot*x_dot - r_dot*y_dot - g*cos(phi)*cos(theta) - (0.25*z_dot)/m;                    
  
    % Accelerations
    %z_ddot = F_z/m - g;
    r_ddot =  (T_x + Izz*p_dot*s_dot - Iyy*p_dot*s_dot)/Ixx;
    p_ddot =  (T_y + Ixx*r_dot*s_dot - Izz*r_dot*s_dot)/Iyy;
    s_ddot =  (T_z + Iyy*p_dot*r_dot - Ixx*p_dot*r_dot)/Izz;

    B = P1*[r_dot;p_dot;s_dot];
    phi_dot = B(1);
    theta_dot = B(2);
    psi_dot = B (3);
    % Return state derivatives
    dXdt = [x_dot; y_dot; z_dot; x_ddot; y_ddot; z_ddot; phi_dot; theta_dot; psi_dot; r_ddot; p_ddot; s_ddot; phi_ref_dot; theta_ref_dot];
end 


