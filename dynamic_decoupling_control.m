

function tau = dynamic_decoupling_control(q, q_dot, q_d, q_dot_d, q_ddot_d, Kp, Kv, robot, error_factors)

    [M_true, V_true, G_true] = getRobotDynamics(q, q_dot, robot);

    M_hat = error_factors.M * M_true;
    V_hat = error_factors.V * V_true;
    G_hat = error_factors.G * G_true;
    
    e = q_d - q;
    e_dot = q_dot_d - q_dot;
    tau_prime = q_ddot_d + Kv * e_dot + Kp * e;

    tau = M_hat * tau_prime + V_hat + G_hat;
end