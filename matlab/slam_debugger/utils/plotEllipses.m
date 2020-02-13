function plotEllipses(mean,std,th)
     t = linspace(0,2*pi,100);
     a = std(1);
     b = std(2);
     x0 = mean(1);
     y0 = mean(2);
     x = x0 + a*cos(t)*cos(th) - b*sin(t)*sin(th);
     y = y0 + b*sin(t)*cos(th) + a*cos(t)*sin(th);
     plot(x,y);
     axis equal;
end