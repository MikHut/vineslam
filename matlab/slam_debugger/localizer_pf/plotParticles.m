function plotParticles(particles, gt)
    figure(1)
    hold on
    grid on
    xlim([0, 300])
    ylim([-50, 50])
    
    for i = 1:length(particles)
        plot(particles(i).pose(1), particles(i).pose(2), 'ro', 'MarkerSize', 2, 'LineWidth', 2);
    end
    plot(gt(1), gt(2), 'bo', 'MarkerSize', 2, 'LineWidth', 2);
end

