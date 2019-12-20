close all
clc
clear

% load functions
addpath('utils');
addpath('mapper_kf');

% GLOBAL VARS
global h_fov
global v_fov
global img_w
global img_h

h_fov = pi/2;
v_fov = pi/2;
img_w = 1292;
img_h = 964;

% input vars
obsv   = csvread('/home/andre-criis/Source/agrobvslam/matlab/slam_debugger/data/image_pos.csv', 0, 0);
iters  = csvread('/home/andre-criis/Source/agrobvslam/matlab/slam_debugger/data/iterations.csv', 0, 0);
r_pose = csvread('/home/andre-criis/Source/agrobvslam/matlab/slam_debugger/data/poses.csv', 0, 0);

% iniliaze covariance
P = [0.3, 0;
     0, 0.4];
std_th = 1;
std_y  = 1;

% perform first iteration - find a correspondence between the landmark
% observation on image and the initial map
j     = 144;
iters = nonzeros(iters(j,:));
obsv  = nonzeros(obsv(j,:));

robot_x_   = r_pose(iters(:)+1,1);
robot_y_   = r_pose(iters(:)+1,2);
robot_th_  = r_pose(iters(:)+1,3);

k  = 10;
l  = kf_init(r_pose(iters(1:k)+1,:), obsv(1:k));
l_ = l';

% loop vars
inc   = 1; 
N     = length(obsv);

figure(1)
hold on
grid on
axis equal
figure(2)
hold on
grid on
axis equal
% loop
for i = (inc + 1):(N-1)
    p_robot_x  = robot_x_(i -  inc);
    p_robot_y  = robot_y_(i -  inc);
    p_robot_th = robot_th_(i -  inc);
    c_robot_x  = robot_x_(i);
    c_robot_y  = robot_y_(i);
    c_robot_th = robot_th_(i);
    
    % calculation of the observation using the detection and the robot
    % movement
    c_col   = obsv(i);
    p_col   = obsv(i-inc);
    delta_p = [c_robot_x - p_robot_x, c_robot_y - c_robot_y, c_robot_th - p_robot_th]';
    z       = processObsv(p_col, c_col, delta_p);
    
    % computation of the covariance of the observation
    % - dependes on if it is/isnt inside the camera field of view
    % - displacement to the predicted y value
    x  = z(1) * cos(z(2)) + p_robot_x;
    y  = z(1) * sin(z(2)) + p_robot_y;
    th = z(2);
    abs_min_thtol = -h_fov/2 + p_robot_th;
    abs_max_thtol = +h_fov/2 + p_robot_th;
    is_inside = ~(th < abs_min_thtol || th > abs_max_thtol || norm([p_robot_x, p_robot_y] - [x,y]) > 4);
    
    dist_y   = (y - l_(2));
    dist_th  = NormalizeAng(atan2(y,x) - atan2(l_(2),l_(1)));
    R = [~is_inside*1000 + std_y * (dist_y * dist_y), 0;
         0, ~is_inside*1000 + std_th*(dist_th*dist_th)];
    
    % kalman filter invocation
    r = [p_robot_x, p_robot_y, p_robot_th]';
    [l,P] = kf(r,l,z,P,R);
    X(i -  inc,:) = l;
    figure(1)
    plot(l(1), l(2), 'ro', 'MarkerSize', 2, 'LineWidth', 2);
    plot(p_robot_x, p_robot_y, 'ko', 'MarkerSize', 2, 'LineWidth', 2);
    
    figure(2)
    hold on
    plot(x, y, 'ko', 'MarkerSize', 2, 'LineWidth', 2);
    Y(i - inc,:) = [x,y];
end

std_x = std(X(:,1));
std_y = std(X(:,2));
mean_x = mean(X(:,1));
mean_y = mean(X(:,2));
figure(1)
plotEllipses([mean_x,mean_y], [std_x,std_y]);

std_x = std(Y(:,1));
std_y = std(Y(:,2));
mean_x = mean(Y(:,1));
mean_y = mean(Y(:,2));
figure(2)
plotEllipses([mean_x,mean_y], [std_x,std_y]);