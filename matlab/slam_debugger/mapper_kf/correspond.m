function l = correspond(r,col,L)
    % this function finds the correspondence between an observation of a
    % landmark on the image and its location on the map
    % - r = [r_x,r_y,r_z]: robot pose
    % - col: observation of the landmark on the image (column)
    % - L = [l1, l2, ..., ln]: map of landmarks
    
    th   = columnToTheta(col);
    line = Line(r(1), r(2), 10 * cos(th), 10 * sin(th));
    plotLine(line, [r(1)-20, r(2)-10], [r(1) + 20, r(2) + 10]);
    
    y_length = L(2,1,2) - L(1,1,2);
    min = 1e6;
    for i = 1:size(L,1)
        for j = 1:size(L,2)
            if abs(r(2) - L(i,j,2)) < y_length && (L(i,j,1) - r(1)) > 0
                d = line.dist(L(i,j,:));
                if d < min
                    min = d;
                    l = [L(i,j,1), L(i,j,2)]';
                end
            end
        end
    end
    
    plot(l(1), l(2), 'ro', 'MarkerSize', 2, 'LineWidth', 2);
end