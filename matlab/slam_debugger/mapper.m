close all
clc
clear

% GLOBAL VARS
global h_fov
global v_fov
global img_w
global img_h

h_fov = pi/4;
v_fov = pi/4;
img_w = 1292;
img_h = 964;

% READ INPUT DATA
M      = csvread('/home/andre/Source/matlab/odometer_debuger/data/single_trunk.csv', 0, 0);
r_pose = csvread('/home/andre/Source/matlab/odometer_debuger/data/robot_pose.csv',   0, 0);
robot_x_raw  = r_pose(:,1) .* 100;
robot_y_raw  = r_pose(:,2) .* 100;
robot_th_raw = r_pose(:,3);

% FILTER ROBOT POSE
robot_x  = robot_x_raw;
robot_y  = robot_y_raw;
robot_th = robot_th_raw;
for i=5:158
    robot_x(i)  = (robot_x(i)  + robot_x(i-1)  + robot_x(i-2)  + robot_x(i-3)  + robot_x(i-4))  / 5; 
    robot_y(i)  = (robot_y(i)  + robot_y(i-1)  + robot_y(i-2)  + robot_y(i-3)  + robot_y(i-4))  / 5;
    robot_th(i) = (robot_th(i) + robot_th(i-1) + robot_th(i-2) + robot_th(i-3) + robot_th(i-4)) / 5;
end   

figure(1)
plot(robot_x, robot_y)
hold on
plot(robot_x_raw, robot_y_raw);

% LOCAL VARS
inc = 50; % window increment
it  = 1;  % loop iterator
k   = 0;  % var to calculate the estimation average
i   = 1;  % .csv column to consider
fr  = -5;
% MAIN LOOP

figure(4)
plot(0, 0, 'ko', 'MarkerSize', 5, 'LineWidth', 5); % draw circle on origin
hold on

l_img  = M(1:158,i);
all_theta = -(h_fov / img_w) * (img_w / 2 - l_img);

figure(3)
hold on
plot(all_theta);

index = length(l_img);
avg = [0, 0];
for j = (inc + 1 - fr):it:(index - 1)

    X_prev = l_img(j -  inc);
    X_curr = l_img(j);
    
    delta_x  = robot_x(j + fr)  - robot_x(j -  inc + fr);
    delta_y  = robot_y(j + fr)  - robot_y(j -  inc + fr);
    delta_th = robot_th(j + fr) - robot_th(j -  inc + fr);
    
    th(j)  = robot_th(j + fr);
    cor(j) = all_theta(j) - robot_th(j -  inc + fr);
    vx(j)  = delta_x;
  
    % ROBOT POSE
    p_robot_px = robot_x(j -  inc + fr);
    p_robot_py = robot_y(j -  inc + fr);
    p_robot_th = robot_th(j -  inc + fr);

    c_robot_px = robot_x(j + fr);
    c_robot_py = robot_y(j + fr);
    c_robot_th = robot_th(j + fr);

    % CALCS
    l_prev = computeLine(X_prev, 0);
    l_proj = projectLine(X_curr, [delta_x, delta_y], -delta_th);
    [x, y] = intercept(l_prev, l_proj);
    X(j) = x;
    Y(j) = y;

    trunk_xy = [p_robot_px + x, p_robot_py + y];

    % VISUALIZATION
    figure(4)
    hold on
    plotLine(l_prev);
    plotLine(l_proj);
    plot(x, y, '*', 'MarkerSize', 10, 'LineWidth', 1);

    fprintf('it: %d\n  X_prev = %d\n  X_curr = %d\n', i, X_prev, X_curr);
    fprintf('  [r_x, r_y r_th]_p  = [%.4f, %.4f, %.4f]\n', p_robot_px, p_robot_py, p_robot_th);
    fprintf('  [r_x, r_y r_th]_c  = [%.4f, %.4f, %.4f]\n', c_robot_px, c_robot_py, c_robot_th);        
    fprintf('  [d_x, d_y, d_th]   = [%.4f, %.4f, %.4f]\n', delta_x, delta_y, delta_th);
    fprintf('  [trunk_x, trunk_y] = [%.4f, %.4f]\n', p_robot_px + x, p_robot_py + y);

    figure(2)
    plot(p_robot_px, p_robot_py, 'ko', 'MarkerSize', 5, 'LineWidth', 5);
    hold on
    plot(trunk_xy(1), trunk_xy(2), 'r*', 'MarkerSize', 5, 'LineWidth', 5);
    grid on
    
    avg = (trunk_xy + (avg * k)) / (k + 1);
    k = k + 1;

    it = it + 1;
end
figure(2)
plot(avg(1), avg(2), 'g*', 'MarkerSize', 5, 'LineWidth', 5);

figure(3)
plot(th); hold on
plot(cor); 

std_x = std(X);
std_y = std(Y);