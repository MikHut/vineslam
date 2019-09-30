function plotLine(l)
    x_min = -750;
    x_max = 750;
    n = 100;
    x = linspace(x_min,x_max, n);
    y = (l.c - l.a * x) / l.b;
    
    plot(x,y);
    %xlim([x_min, x_max])
    ylim([-750, 750])
    grid on   
end

