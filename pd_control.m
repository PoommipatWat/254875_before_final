

function tau = pd_control(q, q_dot, q_d, Kp, Kv)

    tau = -Kp * (q - q_d) - Kv * q_dot;
    
end