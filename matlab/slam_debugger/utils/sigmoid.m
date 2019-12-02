function y = sigmoid(x, is_inside)
    % this function is a sigmoid
    y = x / (1 + abs(x) * 10); 
    y = y * (~is_inside * 1000);
end

