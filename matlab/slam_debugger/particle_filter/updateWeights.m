function particles = updateWeights(particles)
    w_sum = 0;
    for i = 1:length(particles)
        if particles(i).r_error == 1e6
            particles(i).w = 0;
        else
            particles(i).w = exp(-0.5 * particles(i).r_error);
        end
        w_sum = w_sum + particles(i).w;
    end
    
    for i = 1:length(particles)
        if w_sum > 0
            particles(i).w = particles(i).w / w_sum;
        else
            particles(i).w  = 0;
        end
    end
end