
function J = jacobian(q, robot)

    % ดึงค่าพารามิเตอร์ที่จำเป็น
    L1 = robot.L1;
    L2 = robot.L2;
    
    % ดึงค่ามุม
    th1 = q(1);
    th2 = q(2);
    
    % คำนวณค่า sin/cos ที่ต้องใช้
    s1 = sin(th1);
    s12 = sin(th1 + th2);
    c1 = cos(th1);
    c12 = cos(th1 + th2);
    
    % คำนวณส่วนประกอบแต่ละตัวของ Jacobian
    j11 = -L1*s1 - L2*s12;
    j12 = -L2*s12;
    j21 = L1*c1 + L2*c12;
    j22 = L2*c12;
    
    % เมทริกซ์ J
    J = [j11, j12;
         j21, j22];
end