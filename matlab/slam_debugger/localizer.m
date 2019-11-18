close all
clc
clear

% load functions
addpath('utils');
addpath('particle_filter');

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
trunk_pos = [33 -7;
             20, 4;
             69, -5;
             65, -6;
             72, 2];

% particle filter vars
n_particles = 100;
mean = [0 0 0];
std  = [1 1 0.01]; % [x y] in dm and theta in radians

% init particles
particles = init(n_particles, mean, std);
plotParticles(particles);

% loop
for it = 1:size(M,1)
    cols = M(it,:);
    particles = predict(particles, cols, trunk_pos, mean);
    particles = updateWeights(particles);
    particles = resample(particles);
    
    % update particles mean
    for i = 1:length(particles)
        mean(1) = mean(1) + particles(i).pose(1);
        mean(2) = mean(2) + particles(i).pose(2);
        mean(3) = mean(3) + particles(i).pose(3);
    end
    mean(1) = mean(1) / length(particles);
    mean(2) = mean(2) / length(particles);
    mean(3) = mean(3) / length(particles);    
    
    plotParticles(particles);
    pause();
 end