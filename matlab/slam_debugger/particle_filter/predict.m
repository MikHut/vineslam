function particles = predict(in_particles, img_pos, world_pos, mean)
    global img_w;

    x_mean  = 0;
    y_mean  = 0;
    th_mean = 0;
    
    % recompute particles with last average
    for i = 1:length(in_particles)
        x_mean  = x_mean  + in_particles(i).pose(1);
        y_mean  = y_mean  + in_particles(i).pose(2);
        th_mean = th_mean + in_particles(i).pose(3);
    end
    x_mean  = x_mean  / length(in_particles);
    y_mean  = y_mean  / length(in_particles);
    th_mean = th_mean / length(in_particles);
    
    vel = [x_mean - mean(1), y_mean - mean(2), th_mean - mean(3)] .* 10;
    
    particles = normal_d(in_particles, [x_mean + vel(1), y_mean + vel(2), th_mean + vel(3)], vel);
    
    % prediction loop
    for i = 1:length(particles)
        n_corr = 0;
        for j = 1:size(world_pos, 1)
            m_world_pos = world_pos(j,:);
            pt = [m_world_pos(1) - particles(i).pose(1), -(m_world_pos(2) + particles(i).pose(2))];
            
            % check if landmark is inside the field of view
            th = atan2(pt(2), pt(1));
            abs_min_thtol = columnToTheta(0) + particles(i).pose(3);
            abs_max_thtol = columnToTheta(img_w) + particles(i).pose(3);
            if th < abs_min_thtol || th > abs_max_thtol
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
                if r_error < 15
                    n_corr  = n_corr + 1;
                    particles(i).r_error = particles(i).r_error + r_error;
%                    particles(i).pose
                end
            end
            if n_corr == 0
                particles(i).r_error = 1e6;
            else
                particles(i).r_error = particles(i).r_error / n_corr;
            end
        end
    end
end