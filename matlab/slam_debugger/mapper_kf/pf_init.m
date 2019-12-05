function p = pf_init(side)
    % this function initializes the particle filter
    % - side: trunk is on the left (0) or rigth (1) from the robot
    
    if side == 1
        x = 0.1:0.05:1.5;
    else
        x = -1.5:0.05:-0.1;
    end
    
    N = length(x);
    for i = 1:N
        p(i) = Particle(i,x(i));
    end
end

