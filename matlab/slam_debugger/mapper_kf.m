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

% iniliaze map and covariance of the map
L = init(100,4); % array that contains the observations for the EKF
P = [0.3, 0;
     0, 0.4];
plotMap(L, [0,0], P);
std_th = 25;
std_y  = 4;
pause()
% perform first iteration - find a correspondence between the landmark
% observation on image and the initial map
j     = 8;
iters = nonzeros(iters(j,:));
obsv  = nonzeros(obsv(j,:));
l_    = correspond([sum(r_pose(iters(1:4)+1,1))/4; sum(r_pose(iters(1:4)+1,2))/4; sum(r_pose(iters(1:4)+1,3))/4], sum(obsv(1:4)/4), L)
l = l_;

% filter the robot pose
filter_dim = 5;
robot_x_   = r_pose(iters(:)+1,1);
robot_y_   = r_pose(iters(:)+1,2);
robot_th_  = r_pose(iters(:)+1,3);
for i=5:length(iters)
    robot_x_(i)  = (robot_x_(i)  + robot_x_(i-1)  + robot_x_(i-2)  + robot_x_(i-3)  + robot_x_(i-4))  / 5; 
    robot_y_(i)  = (robot_y_(i)  + robot_y_(i-1)  + robot_y_(i-2)  + robot_y_(i-3)  + robot_y_(i-4))  / 5;
    robot_th_(i) = (robot_th_(i) + robot_th_(i-1) + robot_th_(i-2) + robot_th_(i-3) + robot_th_(i-4)) / 5;
end   

% loop vars
inc   = 50; 
N     = length(obsv);

figure(2)
hold on
grid on
% loop
for i = (inc + 1 + filter_dim):(N-1)
    p_robot_x  = robot_x_(i -  inc - filter_dim);
    p_robot_y  = robot_y_(i -  inc - filter_dim);
    p_robot_th = robot_th_(i -  inc - filter_dim);
    c_robot_x  = robot_x_(i - filter_dim);
    c_robot_y  = robot_y_(i - filter_dim);
    c_robot_th = robot_th_(i - filter_dim);
    
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
    abs_min_thtol = -h_fov/2 + c_robot_th;
    abs_max_thtol = +h_fov/2 + c_robot_th;
    is_inside = ~(th < abs_min_thtol || th > abs_max_thtol || norm([p_robot_x, p_robot_y] - [x,y]) > 4);
    
    dist_y   = (y - l_(2));
    dist_th  = NormalizeAng(atan2(y,x) - atan2(l_(2),l_(1)));
    R = [~is_inside*1000 + std_y * (dist_y * dist_y), 0;
         0, ~is_inside*1000 + std_th*(dist_th*dist_th)];
    
    % kalman filter invocation
    r = [p_robot_x, p_robot_y, p_robot_th]';
    [l,P] = kf(r,l,z,P,R);
    X(i -  inc - filter_dim,:) = l;
    figure(1)
    plot(l(1), l(2), 'ro', 'MarkerSize', 2, 'LineWidth', 2);
    plot(p_robot_x, p_robot_y, 'ko', 'MarkerSize', 2, 'LineWidth', 2);
    
    figure(2)
    hold on
    plot(x, y, 'ko', 'MarkerSize', 2, 'LineWidth', 2);
    Y(i - inc - filter_dim,:) = [x,y];
 
    %pause();
end

std_x = std(X(:,1));
std_y = std(X(:,2));
mean_x = mean(X(:,1));
mean_y = mean(X(:,2));
figure(1)
plot(std_x*cos(0:0.01:2*pi)+mean_x, std_y*sin(0:0.01:2*pi)+mean_y, 'ko', 'MarkerSize', 1, 'LineWidth', 1);

std_x = std(Y(:,1));
std_y = std(Y(:,2));
mean_x = mean(Y(:,1));
mean_y = mean(Y(:,2));
figure(2)
plot(std_x*cos(0:0.01:2*pi)+mean_x, std_y*sin(0:0.01:2*pi)+mean_y, 'ko', 'MarkerSize', 1, 'LineWidth', 1);