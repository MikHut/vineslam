function particles = normal_d(particles, mean, std)  
    n_particles = length(particles);
    
    rng(0,'twister');
    
    mean_x = mean(1);
    std_x  = std(1);
    x_ = std_x .* randn(n_particles, 1) + mean_x;  
    
    mean_y = mean(2);
    std_y  = std(2);
    y_ = std_y .* randn(n_particles, 1) + mean_y;  
    
    mean_th = mean(3);
    std_th  = std(3);
    th_ = std_th .* randn(n_particles, 1) + mean_th;
    
    for i = 1:n_particles
        particles(i).pose(1) = x_(i);
        particles(i).pose(2) = y_(i);
        particles(i).pose(3) = th_(i);
    end
end