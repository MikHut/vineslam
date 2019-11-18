function particles = init(n_particles, mean, std)
    for i = 1:n_particles
        p = Particle(i,0,0,0,1);
        particles(i) = p;
    end
    
    [particles] = normal_d(particles, mean, std);
end

