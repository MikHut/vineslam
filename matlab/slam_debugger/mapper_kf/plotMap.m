function plotMap(L,r,P)
    % this function plots the landmarks on the map
    
    figure(1)
    xlim([-40, 40])
    ylim([-20, 20])
    hold on
    grid on
    
    plot(r(1), r(2), 'ko', 'MarkerSize', 3, 'LineWidth', 3);
    for i = 1:size(L,1)
        for j = 1:size(L,2)
            plot(L(i,j,1), L(i,j,2), 'bo', 'MarkerSize', 2, 'LineWidth', 2);
            plot(P(1,1)*cos(0:0.01:2*pi)+L(i,j,1), P(2,2)*sin(0:0.01:2*pi)+L(i,j,2), 'bo', 'MarkerSize', 1, 'LineWidth', 1);
        end
    end
end