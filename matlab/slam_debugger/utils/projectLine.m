function l_out = projectLine(img_pos, delta_p, delta_th)
    l_aux = computeLine(img_pos, delta_th);

    p1 = l_aux.p1 + delta_p;
    p2 = l_aux.p2 + delta_p;
    
    l_out = Line(p1(1), p1(2), p2(1), p2(2));
end

