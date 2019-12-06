function X_curr = predictDetection(X_prev, trunk_y, p_pos)
    % this function predicts the next detection of a trunk using the
    % previous one and the expectable range of possible heights 
    % of the vine
    % - X_prev:  previous prediction
    % - p_pos:   robot position in the previous prediction
    % - trunk_y: expected y distance to the trunk
    % - X_curr:  expected landmark positions
    
    l_prev  = computeLine(X_prev,0);
    l_trunk = Line(0, trunk_y, 10, trunk_y);
    [x,y]   = intercept(l_prev, l_trunk);
    X_curr  = [x,y] + p_pos; 
    
    figure(1);
    hold on;
    plotLine(l_prev, [-1,-5], [30,5]);
    plotLine(l_trunk, [-1,-5], [30,5]);
    plot(x, y, 'ko', 'MarkerSize', 2, 'LineWidth', 2);
    plot(X_curr(1), X_curr(2), 'ko', 'MarkerSize', 2, 'LineWidth', 2);
end

