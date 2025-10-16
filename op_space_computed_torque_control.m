function tau = controller_op_space_computed_torque(q, q_dot, x_d, x_dot_d, x_ddot_d, Kp, Kv, robot_model, error_factors)

    x = forward_kinematics(q, robot_model);
    J = jacobian(q, robot_model);
    x_dot = J * q_dot;

    [M_q_model, V_q_model, G_q_model] = getRobotDynamics(q, q_dot, robot_model);
    
    M_q_hat = error_factors.M * M_q_model;
    V_q_hat = error_factors.V * V_q_model;
    G_q_hat = error_factors.G * G_q_model;

    Mx_hat = inv(J') * M_q_hat * inv(J);
    Gx_hat = inv(J') * G_q_hat;
  
    Vx_hat = inv(J') * V_q_hat - Mx_hat * (zeros(size(J)) * q_dot);


    e = x_d - x;
    e_dot = x_dot_d - x_dot;
    F_prime = x_ddot_d + Kp * e + Kv * e_dot;

    F = Mx_hat * F_prime + Vx_hat + Gx_hat;

    tau = J' * F;
end