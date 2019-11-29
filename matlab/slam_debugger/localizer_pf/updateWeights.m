function particles = updateWeights(particles)
    w_sum = 0;
    for i = 1:length(particles)
        particles(i).w = 1;
        gamma = 0;
        for j = 1:length(particles(i).r_error)
            c      = 1 / (sqrt(2 * pi));
            sigma  = exp(-0.5 * (particles(i).r_error(j) * particles(i).r_error(j)));
            gamma  = exp(-0.5 * (particles(i).r_std(j) * particles(i).r_std(j)));
            particles(i).w = particles(i).w * (c * sigma * gamma);
        end
        if particles(i).w == 1
            particles(i).w = 0;
        end
        w_sum = w_sum + particles(i).w;
    end
    
    for i = 1:length(particles)
        if w_sum == 0
            particles(i).w  = 1/length(particles);
        else
            particles(i).w = particles(i).w / w_sum;
        end
    end
end