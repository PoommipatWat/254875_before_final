
function tau = computed_torque_control(q, q_dot, q_d, q_dot_d, q_ddot_d, Kp, Kv, robot_model, error_factors)

    [M_model, V_model, G_model] = getRobotDynamics(q, q_dot, robot_model);

    M_hat = error_factors.M * M_model;
    V_hat = error_factors.V * V_model;
    G_hat = error_factors.G * G_model;
    
    e = q_d - q;
    e_dot = q_dot_d - q_dot;
    tau_prime = q_ddot_d + Kv * e_dot + Kp * e;
  
    tau = M_hat * tau_prime + V_hat + G_hat;
end