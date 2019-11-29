close all
clc
clear

% load functions
addpath('utils');
addpath('localizer_pf');

% GLOBAL VARS
global h_fov
global v_fov
global img_w
global img_h

h_fov = pi/2;
v_fov = pi/2;
img_w = 1292;
img_h = 964;

% read input vars
M = csvread('/home/andre-criis/Source/agrobvslam/matlab/slam_debugger/data/detections.csv', 0, 0);
GT = csvread('/home/andre-criis/Source/agrobvslam/matlab/slam_debugger/data/poses.csv', 0, 0);
trunk_pos = [37, -15;
             74, -10;
             68, -10
             99,   3;
             120, -9;
             130,  5;
             131, -6;
             144,  6;
             179,  9;
             199,  6;
             224,  6;
             232,  6;
             331, -12;
             341, -15,
             358,  -9;
             407,  16;
             467, -19];
              
        
% particle filter vars
n_particles = 50;
c_mean = [0 0 0];
p_mean = [0 0 0];
std    = [0.5 0.5 0.01]; % [x y] in dm and theta in radians
n_obsv = 0;

% init particles
particles = init(n_particles, c_mean, std);
plotParticles(particles,[0,0,0]);

% loop
gt_last = [0,0,0];
for it = 1:size(M,1)
    cols    = M(it,:);
    gt      = GT(it,:) .* 10;
    vt_real = gt - gt_last;
    gt_last = gt;

    vel = [norm(vt_real(1:2)), 0, 0];
    mag = vel .* 1;
    if n_obsv == 0
        mag = [2,2,0.1];
    end

    % prediction
    [particles, n_obsv] = predict(particles, cols, trunk_pos, vel, mag);
    particles = updateWeights(particles);
    if n_obsv > 0
        particles = resample(particles);
    end 
    
    n_obsv
    % update particles mean
    for i = 1:length(particles)
        c_mean(1) = c_mean(1) + particles(i).pose(1);
        c_mean(2) = c_mean(2) + particles(i).pose(2);
        c_mean(3) = c_mean(3) + particles(i).pose(3);
    end
    c_mean(1) = c_mean(1) / length(particles);
    c_mean(2) = c_mean(2) / length(particles);
    c_mean(3) = c_mean(3) / length(particles);
    
    % calculate estimation error
    fprintf('real_p = [%.4f, %.4f, %.4f]\n', gt(:));
    fprintf('estm_p = [%.4f, %.4f, %.4f]\n', c_mean(:));
    fprintf('error  = [%.4f, %.4f, %.4f]\n', gt(:) - c_mean(:));
    
    % plots
    plotParticles(particles, gt);
    pause(0.01);
    clf('reset');
    clc

    % save mean of last iter
    p_mean    = c_mean;
    c_mean(:) = 0;
 end