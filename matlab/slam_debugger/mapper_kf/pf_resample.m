function p = pf_resample(particles)
    % this function implements the resample step of the particle filter    

    for i = 1:length(particles)
        w(i) = particles(i).w;
    end

    M = length(w);
    Q = cumsum(w);

    Q(M) = 1;

    i=1;
    indx = zeros(M);
    while (i<=M)
        sampl = rand(1,1);  % (0,1]
        j=1;
        while (Q(j)<sampl)
            j=j+1;
        end
        indx(i)=j;
        i=i+1;
    end

    for i = 1:M
        particles(i).dy  = particles(indx(i)).dy + (rand(1) -  0.5) * 0.01;
        particles(i).w   = particles(indx(i)).w;
        particles(i).cov = particles(indx(i)).cov;    
    end    
    
    p = particles;
    
end

