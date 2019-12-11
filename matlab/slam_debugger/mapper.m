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

j   = 144;
inc = 1;

iters = nonzeros(iters(j,:));
obsv  = nonzeros(obsv(j,:));

if obsv(1) > img_w/2
    p = pf_init(1);
else
    p = pf_init(0);
end

% iniliaze covariance
P = [0.3, 0;
     0, 0.4];

for i = inc+1:length(obsv)
    c_pose = r_pose(iters(i)+1,:);
    p_pose = r_pose(iters(i-inc)+1,:);
    c_obsv = obsv(i);
    p_obsv = obsv(i-inc);
    
    [p, TH, P] = pf_predict(p, c_pose, p_pose, c_obsv, p_obsv, P);
    P = [0.3, 0;
         0, 0.4];
    
    if p(1).cov < 1e3
        p = pf_update(p);
        p = pf_resample(p);
    end
    
    pause(0.5);
    clf('reset');
end