

function x = forward_kinematics(q, robot)

    % ดึงค่าพารามิเตอร
    L1 = robot.L1;
    L2 = robot.L2;
    
    % ดึงค่ามุม
    th1 = q(1);
    th2 = q(2);
    
    % คำนวณค่า sin/cos ที่ต้องใช้
    c1 = cos(th1);
    s1 = sin(th1);
    c12 = cos(th1 + th2);
    s12 = sin(th1 + th2);
    
    % คำนวณตำแหน่ง x และ y
    x_pos = L1*c1 + L2*c12;
    y_pos = L1*s1 + L2*s12;
    
    % คืนค่าเป็นเวกเตอร์
    x = [x_pos; y_pos];
end