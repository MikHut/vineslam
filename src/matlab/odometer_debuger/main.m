close all
clc

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
M = csvread('data/sogrape.csv',0,0);
X_prev   = M(:,1);
X_curr   = M(:,3);
delta_x  = M(:,6);
delta_y  = M(:,7);
delta_th = M(:,8);
match_id = M(:,10);

% LOCAL VARS
n_iters = 141;
n = length(M);
robot_px = 13.3404;
robot_py = -2.33383;

% MAIN LOOP
figure(1);
figure(2);

figure(1)
plot(0, 0, 'ko', 'MarkerSize', 5, 'LineWidth', 5); % draw circle on origin
hold on

it = 1;
for i = 1:n_iters
    robot_px = robot_px + delta_x(it);
    robot_py = robot_py + delta_y(it);
    
    while match_id(it) == i      
        l_prev = computeLine(X_prev(it), 0);
        l_proj = projectLine(X_curr(it), [delta_x(it), delta_y(it)], delta_th(it));
        [x, y] = intercept(l_prev, l_proj);

        figure(1)
        plotLine(l_prev);
        plotLine(l_proj);
        plot(x, y, '*', 'MarkerSize', 10, 'LineWidth', 1);
        
        fprintf('it: %d\n  X_prev = %d\n  X_curr = %d\n', i, X_prev(it), X_curr(it));
        fprintf('  [d_x, d_y, d_th] = [%.4f, %.4f, %.4f]\n', delta_x(it), delta_y(it), delta_th(it) * 180/pi);
        fprintf('  [robot_x, robot_y] = [%.4f, %.4f]\n', robot_px, robot_py);
        fprintf('  [trunk_x, trunk_y] = [%.4f, %.4f]\n', robot_px + x, robot_py + y);

        figure(2)
        plot(robot_px, robot_py, 'ko', 'MarkerSize', 5, 'LineWidth', 5);
        hold on
        plot(robot_px + x, robot_py + y, 'r*', 'MarkerSize', 5, 'LineWidth', 5);
        grid on

       % pause();
        
        it = it + 1;
        
        if it > n
            break
        end
    end
end
