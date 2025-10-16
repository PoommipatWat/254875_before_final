% --- FILENAME: animate_robot.m ---

function animate_robot(time_history, q_history, robot)
% สร้าง Animation การเคลื่อนที่ของหุ่นยนต์ พร้อมแสดงค่ามุม

    figure('Name', 'Robot Animation', 'Position', [100, 100, 800, 600]);
    
    skip_frames = 20;
    
    for i = 1:skip_frames:length(time_history)
        
        theta1 = q_history(1, i);
        theta2 = q_history(2, i);
        
        x0 = 0; y0 = 0;
        x1 = robot.L1 * cos(theta1);
        y1 = robot.L1 * sin(theta1);
        x2 = x1 + robot.L2 * cos(theta1 + theta2);
        y2 = y1 + robot.L2 * sin(theta1 + theta2);
        
        clf;
        hold on;
        
        plot([x0, x1], [y0, y1], 'Color', [0 0.4470 0.7410], 'LineWidth', 6);
        plot([x1, x2], [y1, y2], 'Color', [0.8500 0.3250 0.0980], 'LineWidth', 6);
        plot(x0, y0, 'ok', 'MarkerSize', 15, 'MarkerFaceColor', 'k');
        plot(x1, y1, 'ok', 'MarkerSize', 12, 'MarkerFaceColor', 'w');
        plot(x2, y2, 'ok', 'MarkerSize', 12, 'MarkerFaceColor', 'r');
        
        % --- (ส่วนที่เพิ่มเข้ามา) ---
        % 1. สร้างข้อความสำหรับแสดงมุม (แปลงเป็นองศาเพื่อให้อ่านง่าย)
        angle_text = sprintf('θ₁: %.3f rad\nθ₂: %.3f rad', theta1, theta2);
        
        % 2. แสดงข้อความบนจอ Animation ที่มุมซ้ายบน
        max_reach = robot.L1 + robot.L2;
        text(-max_reach * 0.95, max_reach * 0.95, angle_text, 'FontSize', 12, 'VerticalAlignment', 'top', 'FontWeight', 'bold');
        % --------------------------
        
        hold off;
        
        axis equal;
        grid on;
        xlim([-max_reach-0.2, max_reach+0.2]);
        ylim([-max_reach-0.2, max_reach+0.2]);
        
        title(sprintf('Robot Animation | Time: %.2f s', time_history(i)));
        xlabel('X (m)');
        ylabel('Y (m)');
        
        drawnow;
    end
    
    disp('Animation finished.');
end