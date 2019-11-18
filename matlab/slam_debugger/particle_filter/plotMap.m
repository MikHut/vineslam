function plotMap(particle, w_pos)
    global img_w;

    figure(2)
    hold on
    grid on
    
    plot(w_pos(:,1), w_pos(:,2), 'b*', 'MarkerSize', 2, 'LineWidth', 2);
    plot(particle.pose(1), particle.pose(2), 'ro', 'MarkerSize', 2, 'LineWidth', 2);
    
    l_min = computeLine(0,     particle.pose(3), particle.pose(1:2));
    l_max = computeLine(img_w, particle.pose(3), particle.pose(1:2));
    
    plotLine(l_min, [0 -10], [100 10]);
    plotLine(l_max, [0 -10], [100 10]);
end