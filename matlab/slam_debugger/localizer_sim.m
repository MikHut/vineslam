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

% set input vars
robot_x   = 0:2:100;
robot_y   = zeros(length(robot_x));
robot_th  = zeros(length(robot_x));
trunk_pos = [15,  -6;
             30,  -6;
             45,  -6;
             %60,  -6;
             75,  -6;
             90,  -6;
             105, -6;
             15,  +6;
             %30,  +6;
             45,  +6;
             %60,  +6;
             75,  +6;
             90,  +6;
             105, +6];
         
map_std = (rand(size(trunk_pos,1), 2) - [0.5 0.5]) * 10;

N = length(robot_x);
k = 1;
for i = 1:N
    for j = 1:size(trunk_pos,1)
        pt = (trunk_pos(j,:) + map_std(j,:)) - [robot_x(i), robot_y(i)];
        th = atan2(pt(2), pt(1));
        
        abs_min_thtol = -h_fov/2 + robot_th(i);
        abs_max_thtol = +h_fov/2 + robot_th(i);
        if th < abs_min_thtol || th > abs_max_thtol || norm(trunk_pos(j,:) - [robot_x(i), robot_y(i)]) > 20
            detections(i,j) = 0;
            continue;
        end
        detections(i,j) = thetaToColumn(th);
    end
end

        
% particle filter vars
n_particles = 200;
c_mean = [0 0 0];
p_mean = [0 0 0];
std  = [2 2 0.01]; % [x y] in dm and theta in radians
n_obsv = 0;

% init particles
particles = init(n_particles, c_mean, std);
plotParticles(particles,[0,0,0]);

% loop
gt_last = [0,0,0];
for it = 1:N
    cols = detections(it,:);
    gt   = [robot_x(it), robot_y(it), robot_th(it)];

    vt_real = gt - gt_last;
    gt_last = gt;
    
    vel    = [norm(vt_real(1:2)), 0, 0];
    mag    = vel .* 1;
    mag(3) = 0.1;
    if n_obsv == 0
        mag = [2,2,0.1];
    end

    % prediction
    [particles, n_obsv] = predict(particles, cols, trunk_pos, map_std, vel, mag);
    particles = updateWeights(particles);
    if n_obsv > 0
        particles = resample(particles);
    end  
    
    % update particles mean
    for i = 1:length(particles)
        c_mean(1) = c_mean(1) + particles(i).pose(1);
        c_mean(2) = c_mean(2) + particles(i).pose(2);
        c_mean(3) = c_mean(3) + particles(i).pose(3);
    end
    c_mean(1) = c_mean(1) / length(particles);
    c_mean(2) = c_mean(2) / length(particles);
    c_mean(3) = c_mean(3) / length(particles);
    
%     particles(:).w
    n_obsv
    
    % calculate error from previous estimation
    fprintf('real_p = [%.4f, %.4f, %.4f]\n', gt(:));
    fprintf('estm_p = [%.4f, %.4f, %.4f]\n', c_mean(:));
    fprintf('error  = [%.4f, %.4f, %.4f]\n', gt(:) - c_mean(:));    

    % plots
    plotParticles(particles, gt);
    pause(0.5);
    clf('reset');
    clc

    % save mean of last iter
    p_mean = c_mean;
    c_mean(:) = 0;
 end