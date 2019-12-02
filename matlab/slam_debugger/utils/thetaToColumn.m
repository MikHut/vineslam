function col = thetaToColumn(theta)
    global h_fov
    global img_w
    
    col = (theta + h_fov / 2) * img_w / h_fov;
end

