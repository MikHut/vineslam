function p = pf_update(particles)
    % this function implements the update step of the particle filter
    
    w_sum = 0;
    for i = 1:length(particles)
        particles(i).w = exp(-0.5 * (particles(i).cov * particles(i).cov));
        w_sum = w_sum + particles(i).w;
    end
    
    for i = 1:length(particles)
        particles(i).w = particles(i).w / w_sum;
    end

    p = particles;
end

