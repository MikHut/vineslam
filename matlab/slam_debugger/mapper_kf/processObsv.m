function z = processObsv(p_col, c_col, delta_p)
    % this function estimates an observation of a landmark based on the
    % observed landmark on image and on the robot motion
    % - [p_col, c_col]: match of observations of a landmark on the image
    % - delta_p = [dt_x, dt_y, dt_th]: robot delta pose
    % - z = [sqrt(x^2+y^2), atan2(y,x)]: landmark observation
    
    l_prev = computeLine(p_col, 0);
    l_proj = projectLine(c_col, [delta_p(1), delta_p(2)], -delta_p(3));
    [x,y] = intercept(l_prev, l_proj)
 
    z = [sqrt(x*x + y*y);
         atan2(y,x)];
 end