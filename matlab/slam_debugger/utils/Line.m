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
%         function obj = Line(pts)
%             X  = 0;
%             Y  = 0;
%             XY = 0;
%             X2 = 0;
%             Y2 = 0;
%             
%             for i = 1:size(pts,1)
%                 X  = X + pts(i,1);
%                 Y  = Y + pts(i,2);
%                 XY = XY + pts(i,1) * pts(i,2);
%                 X2 = X2 + pts(i,1) * pts(i,1);
%                 Y2 = Y2 + pts(i,2) * pts(i,2);
%             end
%             
%             X  = X / size(pts,1);
%             Y  = Y / size(pts,1);
%             XY = XY / size(pts,1);
%             X2 = X2 / size(pts,1);
%             Y2 = Y2 / size(pts,1);
%             
%             obj.a = -(XY - X * Y);
%             
%             Bx = X2 - X * X;
%             By = Y2 - Y * Y;
%             
%             if abs(Bx < abs(By))
%                 obj.b = obj.a;
%                 obj.a = By;
%             else
%                 obj.b = Bx;
%             end
%             
%             obj.c = (obj.a * X + obj.b * Y);
%         end
        function y = getY(x)
            y = (l.c - l.a * x) / l.b;
        end
        function d = dist(obj,x)
            den = sqrt((obj.a * obj.a) + (obj.b * obj.b));
            d   = abs(obj.a * x(1) + obj.b * x(2) - obj.c) / den;
        end
    end 
end
