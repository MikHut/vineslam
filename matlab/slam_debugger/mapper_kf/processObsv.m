function z = processObsv(p_col, c_col, delta_p)
    % this function estimates an observation of a landmark based on the
    % observed landmark on image and on the robot motion
    % - [p_col, c_col]: match of observations of a landmark on the image
    % - delta_p = [dt_x, dt_y, dt_th]: robot delta pose
    % - z = [x, y]: landmark observation on the ground
    
    l_prev = computeLine(p_col, 0);
    l_proj = projectLine(c_col, [delta_p(1), delta_p(2)], -delta_p(3));
    [x, y] = intercept(l_prev, l_proj);

    z = [x;
         y];
end