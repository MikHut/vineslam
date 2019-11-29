classdef Line
    % by + ax = c
    % p1 - start point
    % p2 - "end" point
    properties   
        a
        b
        c
        
        p1
        p2
    end
    
    methods
        function obj = Line(p1x, p1y, p2x, p2y)
            obj.a = p2y - p1y;                                                                      
            obj.b = p1x - p2x;
            obj.c = obj.a * p1x + obj.b * p1y;
            
            obj.p1 = [p1x, p1y];
            obj.p2 = [p2x, p2y];
        end
        function y = getY(x)
            y = (l.c - l.a * x) / l.b;
        end
        function d = dist(obj,x)
            den = sqrt((obj.a * obj.a) + (obj.b * obj.b));
            d   = abs(obj.a * x(1) + obj.b * x(2) - obj.c) / den;
        end
    end 
end
