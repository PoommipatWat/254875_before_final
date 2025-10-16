% --- FILENAME: plot_results.m ---

function plot_results(time, q_hist, q_d)
% ฟังก์ชันสำหรับพล็อตกราฟผลลัพธ์การเคลื่อนที่ของข้อต่อ

    figure('Name', 'Simulation Results');
    
    % --- กราฟสำหรับข้อต่อที่ 1 ---
    subplot(2,1,1);
    plot(time, q_hist(1,:), 'b-', 'LineWidth', 2);
    hold on;
    line([time(1) time(end)], [q_d(1) q_d(1)], 'Color', 'r', 'LineStyle', '--', 'LineWidth', 1.5);
    title('Joint 1 Position (θ_1)');
    xlabel('Time (s)');
    ylabel('Angle (rad)');
    legend('Actual Position', 'Desired Position', 'Location', 'SouthEast');
    grid on;
    
    % --- กราฟสำหรับข้อต่อที่ 2 ---
    subplot(2,1,2);
    plot(time, q_hist(2,:), 'b-', 'LineWidth', 2);
    hold on;
    line([time(1) time(end)], [q_d(2) q_d(2)], 'Color', 'r', 'LineStyle', '--', 'LineWidth', 1.5);
    title('Joint 2 Position (θ_2)');
    xlabel('Time (s)');
    ylabel('Angle (rad)');
    legend('Actual Position', 'Desired Position', 'Location', 'SouthEast');
    grid on;
end