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

% INPUT DATA

troncos = [200,100; 400,100; 600,-100; 300,100];

% X_prev   = [900,  1000, 1100]; % previous image x coordinate of landmark
% X_curr   = [1000, 1100, 1200]; % current  image x coordinate of landmark
vx = 50;
vy = 0;
vth = 0;
delta_x = [0, 0, 0];
delta_y = [0, 0, 0];
delta_th = [0, 0, 0];

% LOCAL VARS
n_iters = 5;
robot_px = 0;
robot_py = 0;

% MAIN LOOP
figure(1);
figure(2);

figure(1)
plot(0, 0, 'ko', 'MarkerSize', 5, 'LineWidth', 5); % draw circle on origin
hold on

figure(2)
plot(0, 0, 'ko', 'MarkerSize', 5, 'LineWidth', 5);
hold on

for i = 1:n_iters
    tmp_x = robot_px;
    tmp_y = robot_py;
    
    delta_th(i) = (rand(1) - 0.5) * (10 * pi / 180);
    
    delta_x(i) = vx * cos(delta_th(i));
    delta_y(i) = vx * sin(delta_th(i));
%     delta_th(i) = (rand(1) - 0.5) * (2 * pi / 180);

    
    robot_px = robot_px + delta_x(i);
    robot_py = robot_py + delta_y(i); 
    
    rand_(i) = rand(1) - 0.5;    
    
    m_prev = atan((troncos(4,2) - tmp_y)    / (troncos(4,1) - tmp_x)) + (5*pi/180) * (rand_(i));
    m_curr = atan((troncos(4,2) - robot_py) / (troncos(4,1) - robot_px)) + (5*pi/180) * (rand_(i));
    
    X_prev(i) = m_prev * 646 / (22.5 * pi / 180) + 646
    X_curr(i) = m_curr * 646 / (22.5 * pi / 180) + 646
    
    diff(i) = X_prev(i) - X_curr(i)
    
    
end

figure
plot(diff);
hold on
plot(rand_)

robot_px = 0;
robot_py = 0;
for i = 1:n_iters
    robot_px = robot_px + delta_x(i);
    robot_py = robot_py + delta_y(i); 
    
    % m = atan(troncos(4,1) - robot_px / troncos(4,2) - robot_py); 
    
    delta_x(i) = delta_x(i) + rand(1) * delta_x(i) * 0.1;
    delta_y(i) = delta_y(i) + rand(1) * delta_x(i) * 0.01;
    
    l_prev = computeLine(X_prev(i), 0);
    l_proj = projectLine(X_curr(i), [delta_x(i), delta_y(i)], delta_th(i));
    [x, y] = intercept(l_prev, l_proj);

    figure(1)
    plotLine(l_prev);
    plotLine(l_proj);
    plot(x, y, '*', 'MarkerSize', 10, 'LineWidth', 1);

    fprintf('it: %d\n  X_prev = %d\n  X_curr = %d\n', i, X_prev(i), X_curr(i));
    fprintf('  [d_x, d_y, d_th]   = [%.4f, %.4f, %.4f]\n', delta_x(i), delta_y(i), delta_th(i) * 180/pi);
    fprintf('  interception       = [%.4f, %.4f]\n', x, y);
    fprintf('  [robot_x, robot_y] = [%.4f, %.4f]\n', robot_px, robot_py);
    fprintf('  [trunk_x, trunk_y] = [%.4f, %.4f]\n', robot_px - delta_x(i) + x, robot_py - delta_y(i) + y);

    figure(2)
    plot(robot_px, robot_py, 'ko', 'MarkerSize', 5, 'LineWidth', 5);
    plot(robot_px - delta_x(i) + x, robot_py - delta_y(i) + y, 'r*', 'MarkerSize', 5, 'LineWidth', 5);
    grid on   
    
    pause();
end
