function [l,P] = kf(r,l,z,P,R)
    % this function is a kalman filter to estimate the position of a single
    % landmark
    % - r = [r_x,r_y,r_th]: robot pose
    % - l = [l_x,l_y]: landmark position - previous state estimation
    % - z = [z_x, z_y]: observation of landmark (calculated with line
    % interception
    % - P: covariance of the previous state estimation
    % - R: covariance of the previous observation

    [l,P] = predict(l,P);
    [l,P] = correct(r,l,z,P,R);
end

