classdef Particle

    properties
        id
        pose    % particle pose
        last_pose 
        vel     % velocity
        w       % weight
        r_error % reprojection error
        r_std   % reprojection standard deviation
    end
    
    methods
        function obj = Particle(id, x, y, th, w)
            obj.id      = id;
            obj.pose    = [x y th];
            obj.last_pose = [x y th];
            obj.vel = [0 0 0];
            obj.w       = w;
            obj.r_error = [0];
            obj.r_std   = [0];
        end
    end
end

