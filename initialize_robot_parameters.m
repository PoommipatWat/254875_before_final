

disp('Initializing robot parameters...');
robot = struct();

% link 1
robot.m1 = 1.0;
robot.L1 = 1.0;   
robot.lc1 = 0.5;
robot.I1 = 0.1;

% link 2
robot.m2 = 1.0;
robot.L2 = 1.0;    
robot.lc2 = 0.5;
robot.I2 = 0.1;

% g
robot.g = 9.81;