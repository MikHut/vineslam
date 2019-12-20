function X = kf_init(robot_p, obsv)
    M = length(obsv);
    
    robot_x  = robot_p(:,1);
    robot_y  = robot_p(:,2);
    robot_th = robot_p(:,3);

    % construct T*S = m linear system 
    T       = zeros(2*M, M+2);
    local_t = zeros(2,   M+2);
    m       = zeros(2*M,   1);
    j       = 1;
    for i = 1:2:2*M
        phi = columnToTheta(obsv(j)) + robot_th(j);

        local_t(1, 1:2) = [1, 0];
        local_t(1, j+2) = -cos(phi);
        local_t(2, 1:2) = [0, 1];
        local_t(2, j+2) = -sin(phi);    

        T(i:i+1,:)  = local_t;
        local_t = zeros(2, M+2);

        m(i:i+1, 1) = [robot_x(j), robot_y(j)]'; 

        j = j + 1;
    end

    S = pinv(T'*T)*T'*m;
    
    X = [S(1), S(2)]';
end

