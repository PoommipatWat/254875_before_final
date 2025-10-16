
function tau = hybrid_force_motion_control(q, q_dot, x_d, F_d, F_sensed, Kp_m, Kv_m, Kp_f, Ki_f, robot_model, dt, wall_y_pos)
% Controller ข้อ 5: Unified Motion and Force Control
% เวอร์ชันสมบูรณ์ที่มี 2 โหมดการทำงาน (เข้าหา และ กด)

    % สร้างตัวแปรสำหรับเก็บค่า Integral ของ Force Error
    persistent e_force_int;
    if isempty(e_force_int)
        e_force_int = zeros(2,1);
    end

    % --- 1. ตรวจสอบสถานะและกำหนดโหมดการควบคุม ---
    x = forward_kinematics(q, robot_model);
    
    if x(2) < (wall_y_pos - 0.005) % ถ้ายังไม่ถึงกำแพง
        % โหมด 1: เข้าหา (ใช้ Motion control ทั้งสองแกน)
        Omega = diag([1, 1]);
        F_d_current = [0; 0]; % ยังไม่ควบคุมแรง
        e_force_int = zeros(2,1); % Reset integral term ป้องกัน Windup
    else
        % โหมด 2: กด (ใช้ Hybrid control)
        Omega = diag([1, 0]); % ควบคุมตำแหน่งแกน x, ควบคุมแรงแกน y
        F_d_current = F_d;
    end

    % --- 2. คำนวณส่วนประกอบต่างๆ ---
    J = jacobian(q, robot_model);
    x_dot = J * q_dot;
    
    % ส่วนของ Motion Control (PD control)
    e_motion = x_d - x;
    F_motion = Kp_m * e_motion - Kv_m * x_dot;
    
    % ส่วนของ Force Control (PI control)
    e_force = F_d_current - F_sensed;
    e_force_int = e_force_int + e_force * dt;
    F_force = -Kp_f * e_force - Ki_f * e_force_int;
    
    % --- 3. รวมคำสั่งและชดเชยแรงโน้มถ่วง ---
    F_command = Omega * F_motion + (eye(2) - Omega) * F_force;
    
    [~, ~, G_q_hat] = getRobotDynamics(q, q_dot, robot_model);
    Gx_hat = inv(J') * G_q_hat;
    
    F_final = F_command + Gx_hat;

    % --- 4. แปลงกลับเป็นแรงบิด ---
    tau = J' * F_final;
end