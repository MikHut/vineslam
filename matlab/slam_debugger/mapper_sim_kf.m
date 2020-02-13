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

% INPUT DATA
trunks = [0.2, 0.8;
          0.4, 0.8; 
          0.6, 0.8;
          0.8, 0.8;
          1.0, 0.8;
          0.2,-0.8; 
          0.4,-0.8;
          0.6,-0.8;
          0.8,-0.8;
          1.0,-0.8];
     
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
robot_th = 0;

% MAIN LOOP
figure(1)
grid on
axis equal
hold on

for j = 1:size(trunks,1)

    delta_x  = [0, 0, 0];
    delta_y  = [0, 0, 0];
    delta_th = [0, 0, 0];
    robot_px = 0;
    robot_py = 0;
    robot_th = 0;
    X_prev   = zeros(n_iters);
    X_curr   = zeros(n_iters);
    R        = zeros(n_iters, 3);

    for i = 1:n_iters
        tmp_x = robot_px;
        tmp_y = robot_py;

        delta_th(i) = (rand(1) - 0.5) * (0.1 * pi / 180);
        delta_x(i)  = vx * cos(delta_th(i));
        delta_y(i)  = vx * sin(delta_th(i));

        robot_px = robot_px + delta_x(i);
        robot_py = robot_py + delta_y(i);
        robot_th = robot_th + delta_th(i);

        m_prev = atan((trunks(j,2) - tmp_y)    / (trunks(j,1) - tmp_x))    + (0*pi/180) * ((rand(1) - 0.5)*0);
        m_curr = atan((trunks(j,2) - robot_py) / (trunks(j,1) - robot_px)) + (0*pi/180) * ((rand(1) - 0.5)*0);

        X_prev(i) = thetaToColumn(m_prev);
        X_curr(i) = thetaToColumn(m_curr);

        R(i,:) = [robot_px, robot_py, robot_th];
    end

    P = [0.5, 0;
         0, 0.5];
    std_th = 1;
    std_y  = 1;

    l  = kf_init(R(1:10,:), X_prev(1:10))
    l_ = l;
    
    X = zeros(n_iters,2);
    Y = zeros(n_iters,2);

    p_robot_px = 0;
    p_robot_py = 0;
    p_robot_th = 0;
    c_robot_px = 0;
    c_robot_py = 0;
    c_robot_th = 0;
    for i = 2:n_iters
        p_robot_x  = c_robot_px;
        p_robot_y  = c_robot_py; 
        p_robot_th = c_robot_th;
        p_pose = [p_robot_x, p_robot_y, p_robot_th];

        c_robot_px = c_robot_px + (delta_x(i) + rand(1) * delta_x(i) * 0.01);
        c_robot_py = c_robot_py + (delta_y(i) + rand(1) * delta_x(i) * 0.01); 
        c_robot_th = c_robot_th + delta_th(i);
        c_pose = [c_robot_px, c_robot_py, c_robot_th];

        c_col   = X_curr(i);
        p_col   = X_prev(i);
        z       = processObsv(p_col, c_col, [c_pose - p_pose]);

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
        X(i -  1,:) = l;
        figure(1)
        plot(l(1), l(2), 'ro', 'MarkerSize', 2, 'LineWidth', 2);
        plot(p_robot_x, p_robot_y, 'go', 'MarkerSize', 2, 'LineWidth', 2);

        figure(2)
        grid on
        axis equal
        hold on
        plot(x, y, 'ko', 'MarkerSize', 2, 'LineWidth', 2);
        plot(p_robot_x, p_robot_y, 'go', 'MarkerSize', 2, 'LineWidth', 2);
        Y(i - 1,:) = [x,y];
    end

    th    = atan2(X(end,2)-X(1,2),X(end,1)-X(1,1));
    std_x = std(X(:,1));
    std_y = std(X(:,2));
    mean_x = mean(X(:,1));
    mean_y = mean(X(:,2));
    figure(1)
    plotEllipses([mean_x,mean_y], [std_x,std_y], th);

    th    = atan2(X(end,2)-X(1,2),X(end,1)-X(1,1));
    std_x = std(Y(:,1));
    std_y = std(Y(:,2));
    mean_x = mean(Y(:,1));
    mean_y = mean(Y(:,2));
    figure(2)
    plotEllipses([mean_x,mean_y], [std_x,std_y], th);
end