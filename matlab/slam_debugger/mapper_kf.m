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

% iniliaze map
L = init(50,4); % array that contains the observations for the EKF
plotMap(L, [0,0]);

% perform first iteration - find a correspondence between the landmark
% observation on image and the initial map
robot_x  = r_pose(iters(3)+1,1);
robot_y  = r_pose(iters(3)+1,2);
robot_th = r_pose(iters(3)+1,3);
col      = obsv(1);
l        = correspond([robot_x; robot_y; robot_th], col, L);

% loop vars
iters = iters(3,:);
obsv  = obsv(3,:);
N     = length(obsv);
p_col = col;
p_robot_x  = robot_x;
p_robot_y  = robot_y;
p_robot_th = robot_th;


figure(2)
hold on
grid on
% loop
for i = 2:N
    robot_x  = r_pose(iters(i)+1,1);
    robot_y  = r_pose(iters(i)+1,2);
    robot_th = r_pose(iters(i)+1,3);
    
    % calculation of the observation using the detection and the robot
    % movement
    c_col   = obsv(i);
    delta_p = [robot_x - p_robot_x, robot_y - p_robot_y, robot_th - p_robot_th]';
    z       = processObsv(p_col, c_col, delta_p);
    
    % computation of the covariance of the observation
    % - dependes on if it is/isnt inside the camera field of view
    % - displacement to the predicted y value
    th = atan2(z(2), z(1));
    abs_min_thtol = -h_fov/2 + robot_th;
    abs_max_thtol = +h_fov/2 + robot_th;
    is_inside = th < abs_min_thtol || th > abs_max_thtol || norm([robot_x, robot_y] - z) > 20;
    
    p_col = c_col;
    p_robot_x  = robot_x;
    p_robot_y  = robot_y;
    p_robot_th = robot_th;

    figure(2)
    hold on
    plot(z(1) + robot_x, z(2) + robot_y, 'ko', 'MarkerSize', 2, 'LineWidth', 2);
end