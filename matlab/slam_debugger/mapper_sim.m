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

h_fov = pi/4;
v_fov = pi/4;
img_w = 1292;
img_h = 964;

% INPUT DATA
trunks = [1, 0.8;
          2, 0.8; 
          1,-0.8; 
          2,-0.8];

j = 4;
      
vx  = 0.005;
vy  = 0;
vth = 0;
delta_x  = [0, 0, 0];
delta_y  = [0, 0, 0];
delta_th = [0, 0, 0];

% LOCAL VARS
n_iters  = 50;
robot_px = 0;
robot_py = 0;

% MAIN LOOP
figure(1)
plot(0, 0, 'ko', 'MarkerSize', 5, 'LineWidth', 5); % draw circle on origin
hold on

for i = 1:n_iters
    tmp_x = robot_px;
    tmp_y = robot_py;
    
    %delta_th(i) = (rand(1) - 0.5) * (10 * pi / 180);
    delta_th(i) = 0;
    delta_x(i)  = vx * cos(delta_th(i));
    delta_y(i)  = vx * sin(delta_th(i));
    
    robot_px = robot_px + delta_x(i);
    robot_py = robot_py + delta_y(i); 
       
    m_prev = atan((trunks(j,2) - tmp_y)    / (trunks(j,1) - tmp_x))    + (5*pi/180) * ((rand(1) - 0.5)*0);
    m_curr = atan((trunks(j,2) - robot_py) / (trunks(j,1) - robot_px)) + (5*pi/180) * ((rand(1) - 0.5)*0);
    
    X_prev(i) = thetaToColumn(m_prev);
    X_curr(i) = thetaToColumn(m_curr);
end

P = [0.3, 0;
     0, 0.4];
 
if X_prev(1) > img_w/2
    p = pf_init(1);
else
    p = pf_init(0);
end

p_robot_px = 0;
p_robot_py = 0;
p_robot_th = 0;
c_robot_px = 0;
c_robot_py = 0;
c_robot_th = 0;
for i = 1:n_iters
    p_robot_px = c_robot_px;
    p_robot_py = c_robot_py; 
    p_robot_th = c_robot_th;
    p_pose = [p_robot_px, p_robot_py, p_robot_th];

    c_robot_px = c_robot_px + (delta_x(i) + rand(1) * delta_x(i) * 0.1);
    c_robot_py = c_robot_py + (delta_y(i) + rand(1) * delta_x(i) * 0.1); 
    c_robot_th = c_robot_th + delta_th(i);
    c_pose = [c_robot_px, c_robot_py, c_robot_th];
       
    [p, TH] = pf_predict(p, c_pose, p_pose, X_curr(i), X_prev(i), P);
    
    if p(1).cov < 1e3
        p = pf_update(p);
        p = pf_resample(p);
    end
       
    pause(0.1);
    clf('reset');
end
