
clear; clc; close all;
initialize_robot_parameters;

% --- Setup ---
robot_model = robot;

% กำหนด error ให้กับ equation of motion
error_factors = struct();
error_factors.M = 1.0;
error_factors.V = 1.0;
error_factors.G = 1.0;

% กำหนด Kp, Kv
Kp = diag([500, 500]);
Kv = diag([200, 200]);

F_d = [0; 0];

t_final = 10;
dt = 0.0001;

% Trajectory
q_d = [pi; pi/2];
q_dot_d = [0; 0];
q_ddot_d = [0; 0];
x_d = forward_kinematics(q_d, robot);
x_dot_d = [0; 0];
x_ddot_d = [0; 0];

% Initial State
q = [0; 0];
q_dot = [0; 0];

time_history = 0:dt:t_final;
q_history = zeros(2, length(time_history));

F_sensed = [0; 0];  % หรือคำนวณจาก dynamics
Kp_m = diag([100, 80]); % Motion gain สำหรับแกน x (ตำแหน่งแรก)
Kv_m = diag([40, 30]);

Kp_f = diag([0, 0]);  % Force gain สำหรับแกน y (ตำแหน่งที่สอง)
Ki_f = diag([0, 0]);  % Integral gain สำหรับ Force

% --- Simulation Loop ---
disp('Starting simulation...');
for i = 1:length(time_history)
    
    % ใน loop:
    

    % >> Controller 1:
    tau = pd_control(q, q_dot, q_d, Kp, Kv);
    
    % >> Controller 2:
    % tau = computed_torque_control(q, q_dot, q_d, q_dot_d, q_ddot_d, Kp, Kv, robot_model, error_factors);

    % >> Controller 3:
    % tau = op_space_pd_control(q, q_dot, x_d, x_dot_d, Kp, Kv, robot_model, error_factors);
    
    % >> Controller 4:
    % tau = op_space_computed_torque_control(q, q_dot, x_d, x_dot_d, x_ddot_d, Kp, Kv, robot_model, error_factors);

    % >> Controller 5: Hybrid Force/Motion Control
    % tau = hybrid_force_motion_control(q, q_dot, x_d, F_d, F_sensed, Kp_m, Kv_m, Kp_f, Ki_f, robot_model, dt);

    % --- Robot Dynamics ---
    [M, V, G] = getRobotDynamics(q, q_dot, robot);
    q_ddot = M \ (tau - V - G);
    
    % --- Integration ---
    q_dot = q_dot + q_ddot * dt;
    q = q + q_dot * dt;
    
    % --- Log Data ---
    q_history(:, i) = q;
end
disp('Simulation finished.');

% --- Plot & Animate ---
plot_results(time_history, q_history, q_d);
animate_robot(time_history, q_history, robot);