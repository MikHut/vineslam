function plotParticles(particles)
    figure(1)
    hold on
    grid on
    
    for i = 1:length(particles)
        plot(particles(i).pose(1), particles(i).pose(2), 'ro', 'MarkerSize', 2, 'LineWidth', 2);
    end
end

