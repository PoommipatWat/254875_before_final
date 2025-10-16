
function [M, V, G] = getRobotDynamics(q, q_dot, robot)

    % ดึงค่าพารามิเตอร์จาก struct
    m1 = robot.m1; m2 = robot.m2;
    L1 = robot.L1;
    lc1 = robot.lc1; lc2 = robot.lc2;
    I1 = robot.I1; I2 = robot.I2;
    g = robot.g;
    
    th1 = q(1); th2 = q(2);
    th1_dot = q_dot(1); th2_dot = q_dot(2);
    
    s2 = sin(th2);
    c1 = cos(th1);
    c2 = cos(th2);
    c12 = cos(th1 + th2);

    % -------------------- คำนวณ Mass Matrix (M) --------------------
    m11 = m1*lc1^2 + I1 + m2*(L1^2 + lc2^2 + 2*L1*lc2*c2) + I2;
    m12 = m2*(lc2^2 + L1*lc2*c2) + I2;
    m22 = m2*lc2^2 + I2;
    M = [m11, m12; m12, m22];

    % ----------------- คำนวณ Coriolis Vector (V) -----------------
    h = -m2 * L1 * lc2 * s2;
    v1 = h * (2 * th1_dot * th2_dot + th2_dot^2);
    v2 = -h * th1_dot^2;
    V = [v1; v2];

    % ------------------ คำนวณ Gravity Vector (G) ------------------
    g1 = (m1*lc1 + m2*L1)*g*c1 + m2*lc2*g*c12;
    g2 = m2*lc2*g*c12;
    G = [g1; g2];
end