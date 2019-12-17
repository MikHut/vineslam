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

obsv   = csvread('/home/andre-criis/Source/agrobvslam/matlab/slam_debugger/data/image_pos.csv', 0, 0);
iters  = csvread('/home/andre-criis/Source/agrobvslam/matlab/slam_debugger/data/iterations.csv', 0, 0);
r_pose = csvread('/home/andre-criis/Source/agrobvslam/matlab/slam_debugger/data/poses.csv', 0, 0);

j   = 618;
inc = 1;

iters    = nonzeros(iters(j,1:20));
obsv     = nonzeros(obsv(j,1:20));
robot_x  = nonzeros(r_pose(iters(1:20)+1, 1));
robot_y  = nonzeros(r_pose(iters(1:20)+1, 2));
robot_th = nonzeros(r_pose(iters(1:20)+1, 3));

M = length(obsv);

% construct T*S = m linear system 
T       = zeros(2*M, M+2);
local_t = zeros(2,   M+2);
m       = zeros(2*M,   1);
j       = 1;
for i = 1:2:2*M
    phi = columnToTheta(obsv(j)) + robot_th(j);
   
    local_t(1, 1:2) = [1, 0];
    local_t(1, j+2) = -cos(phi);
    local_t(2, 1:2) = [0, 1];
    local_t(2, j+2) = -sin(phi);    
    
    T(i:i+1,:)  = local_t;
    local_t = zeros(2, M+2);

    m(i:i+1, 1) = [robot_x(j), robot_y(j)]'; 
    
    j = j + 1;
end

S = pinv(T'*T)*T'*m