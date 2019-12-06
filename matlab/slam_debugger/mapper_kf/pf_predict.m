function [p, TH, P] = pf_predict(particles, c_pose, p_pose, c_col, p_col, P)
    % this function implements the prediction step of the particle filter

    global h_fov;

    p_robot_x  = p_pose(1);
    p_robot_y  = p_pose(2);
    p_robot_th = p_pose(3);
    c_robot_x  = c_pose(1);
    c_robot_y  = c_pose(2);
    c_robot_th = c_pose(3);
    
    M = length(particles);
    
    for i = 1:M
        l(i,:)  = predictDetection(p_col, particles(i).dy, [p_robot_x, p_robot_y])';
        l_(i,:) = l(i,:)';
    end

    % calculation of the observation using the detection and the robot
    % movement
    delta_p = [c_robot_x - p_robot_x, c_robot_y - c_robot_y, c_robot_th - p_robot_th]';
    z       = processObsv(p_col, c_col, delta_p);

    % computation of the covariance of the observation
    % - dependes on if it is/isnt inside the camera field of view
    % - displacement to the predicted y value
    x  = z(1) * cos(z(2)) + p_robot_x;
    y  = z(1) * sin(z(2)) + p_robot_y;
    th = z(2);
    abs_min_thtol = -h_fov/2 + p_robot_th;
    abs_max_thtol = +h_fov/2 + p_robot_th;
    is_inside = ~(th < abs_min_thtol || th > abs_max_thtol || norm([p_robot_x, p_robot_y] - [x,y]) > 4);    
    
    % loop
    for i = 1:M
        TH = l(i,:)';

        % observations covariance matrix calculation
        dist_y   = (y - l_(i,2));
        dist_th  = NormalizeAng(atan2(y,x) - atan2(l_(i,2),l_(i,1)));
        R = [~is_inside*1000 + (dist_y * dist_y), 0;
             0, ~is_inside*1000 + (dist_th * dist_th)];

        % kalman filter invocation
        r = [p_robot_x, p_robot_y, p_robot_th]';
        [TH,P] = kf(r,TH,z,P,R);

        particles(i).cov = norm(R);
        
        figure(1)
        plot(TH(1), TH(2), 'ro', 'MarkerSize', 2, 'LineWidth', 2);
        plot(p_robot_x, p_robot_y, 'ko', 'MarkerSize', 2, 'LineWidth', 2);
    end
    p = particles;
end