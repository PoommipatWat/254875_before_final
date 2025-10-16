

function tau = op_space_pd_control(q, q_dot, x_d, x_dot_d, Kp, Kv, robot_model, error_factors)

    % แปลงเป็น Operational-space
    x = forward_kinematics(q, robot_model);
    J = jacobian(q, robot_model);
    x_dot = J * q_dot;

    % คำนวณแรง F_pd์
    F_pd = -Kp * (x - x_d) - Kv * x_dot;

    % คำนวณ G_hat(x)
    [~, ~, G_q] = getRobotDynamics(q, q_dot, robot_model);
    G_q_hat = G_q * error_factors.G;
    Gx_hat = inv(J') * G_q_hat;

    % คำนวณแรงรวม F ทั้งหมด
    F = F_pd + Gx_hat;

    % แปลงแรง F กลับไปเป็นแรงบิด tau
    tau = J' * F;
end