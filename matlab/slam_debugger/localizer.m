close all
clc
clear

% GLOBAL VARS
global h_fov
global v_fov
global img_w
global img_h

h_fov = pi/2;
v_fov = pi/2;
img_w = 1292;
img_h = 964;

trunk_x  = [20, 33, 65, 69, 72];
trunk_y  = [4, -7, -6, -5, 2];
robot_x  = [10];
robot_y  = [0];
robot_th = [pi/3];

l_min = computeLine(0, robot_th, robot_x);
l_max = computeLine(img_w, robot_th, robot_x);

figure(1)
grid on
hold on

plotLine(l_min);
plotLine(l_max);
plot(robot_x, robot_y, 'o', 'MarkerSize', 10, 'LineWidth', 1);
plot(trunk_x, trunk_y, '*', 'MarkerSize', 10, 'LineWidth', 1);
xlim([0 100]);
ylim([-20 20]);
