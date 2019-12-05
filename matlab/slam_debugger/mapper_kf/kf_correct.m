function [l,P] = kf_correct(r,l,z,P,R)
    % this function the correction step of the Kalman Filter
    % - r = [r_x,r_y,r_th]: robot pose
    % - l = [l_x,l_y]: landmark position - previous state estimation
    % - z = [z_x, z_y]: observation of landmark (calculated with line
    % interception    
    % - P: covariance of the previous state estimation
    % - R: covariance of the previous observation

    
    % observation model
    d   = sqrt((l(1) - r(1)) * (l(1) - r(1)) + (l(2) - r(2)) * (l(2) - r(2)));
    phi = atan2(l(2) - r(2), l(1) - r(1)) - r(3);
    z_ = [d;
          phi];
    z_(2) = NormalizeAng(z_(2));
    
    % jacobian of the observations model in relation with l
    G = [(l(1) - r(1))/d,    (l(2) - r(2))/d;
        -(l(2) - r(2))/(d*d),(l(1) - r(1))/(d*d)];
    
    % kalman gain calculation
    K = P * G' * pinv(G * P * G' + R);
    
    % state correction
    z_diff = (z - z_);
    z_diff(2) = NormalizeAng(z_diff(2));
    l = l + K * z_diff;
    
    % covariance atualization
    P = (eye(2) - K*G) * P;
end