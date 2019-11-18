classdef Particle

    properties
        id
        pose    % particle pose
        w       % weight
        r_error % reprojection error 
    end
    
    methods
        function obj = Particle(id, x, y, th, w)
            obj.id      = id;
            obj.pose    = [x y th];
            obj.w       = w;
            obj.r_error = 0;
        end
    end
end

