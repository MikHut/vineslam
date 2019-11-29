function plotMap(L,r)
    % this function plots the landmarks on the map
    
    figure(1)
    hold on
    grid on
    
    plot(r(1), r(2), 'ko', 'MarkerSize', 3, 'LineWidth', 3);
    for i = 1:size(L,1)
        for j = 1:size(L,2)
            plot(L(i,j,1), L(i,j,2), 'bo', 'MarkerSize', 2, 'LineWidth', 2);
        end
    end
end

