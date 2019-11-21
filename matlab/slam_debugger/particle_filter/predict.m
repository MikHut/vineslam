function [particles, n_obsv] = predict(in_particles, img_pos, world_pos, vel, mag)
    global h_fov;
    
    M = length(in_particles);
    
    % inovate particles
    particles = in_particles;
    for i = 1:M
        particles(i).vel = particles(i).pose - particles(i).last_pose; 

        std = [(rand(1) - .5) * mag(1), (rand(1) - .5)  * mag(2), (rand(1) - .5)  * mag(3)];
        X = in_particles(i).pose;

        particles(i).pose(3) = X(3) + (vel(3) + std(3));
        
        vx = (vel(1) + std(1)) * cos(particles(i).pose(3));
        vy = (vel(1) + std(1)) * sin(particles(i).pose(3));
        particles(i).pose(1) = X(1) + vx;
        particles(i).pose(2) = X(2) + vy;
       
        particles(i).last_pose = particles(i).pose;     
    end
    
    % prediction loop
    n_obsv = 0;
    for i = 1:M
        n_corr = 0;
        particles(i).r_error = 0;
        
        for j = 1:size(world_pos, 1)
            m_world_pos = world_pos(j,:);
            pt = [m_world_pos(1) - particles(i).pose(1), m_world_pos(2) - particles(i).pose(2)];
            
            % check if landmark is inside the field of view
            th = atan2(pt(2), pt(1));
            abs_min_thtol = -h_fov/2 + particles(i).pose(3);
            abs_max_thtol = +h_fov/2 + particles(i).pose(3);
            if th < abs_min_thtol || th > abs_max_thtol || norm(particles(i).pose(1:2) - m_world_pos) > 20
                continue;
            end
            
            % if so, project it on the image
            col = thetaToColumn(th);
            
            % search for correspondences and calculate the reprojection
            % error
            for k = 1:length(img_pos)
                if img_pos(k) == 0
                    continue;
                end
                r_error = sqrt((img_pos(k) - col) * (img_pos(k) - col));
                if r_error < 5
                    n_corr = n_corr + 1;
                    particles(i).r_error = particles(i).r_error + r_error;
                    n_obsv = n_obsv + 1;
                end
            end
        end
        if n_corr == 0
            particles(i).r_error = 1e6;
        else
            particles(i).r_error = particles(i).r_error / (n_corr * n_corr);
        end
    end
end