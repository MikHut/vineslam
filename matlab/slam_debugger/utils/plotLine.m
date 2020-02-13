function plotLine(l, min, max)
    n = 100;
    x = linspace(min(1),max(1), n);
    y = (l.c - l.a * x) / l.b;
    
    plot(x,y);
    xlim([min(1), max(1)])
    ylim([min(2), max(2)])
end

