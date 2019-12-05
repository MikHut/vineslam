classdef Particle
    
    properties
        id  % particle id
        dy  % y displacement from the robot to the trunk
        w   % particle weigth
        cov % covariance of the KF estimation
    end
    
    methods
        function obj = Particle(id,dy)
            obj.id = id;
            obj.dy = dy;
            obj.w  = 1;
        end
    end
    
end

