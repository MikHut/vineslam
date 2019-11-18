function l_out = computeLine(img_pos, delta_th, min)
    global h_fov
    global img_w
    
    theta = -(h_fov / img_w) * (img_w / 2 - img_pos);
    
    p1 = [min(1), min(2)];
    p2 = [20 * cos(theta + delta_th), 20 * sin(theta + delta_th)];
    
    l_out = Line(p1(1), p1(2), p2(1), p2(2));
end
