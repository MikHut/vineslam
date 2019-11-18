function theta = columnToTheta(col)
    global h_fov
    global img_w
    
    theta = (-h_fov / img_w) * (img_w / 2 - col);
end