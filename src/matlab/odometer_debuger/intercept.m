function [px, py] = intercept(obj1, obj2)
    det = obj1.a * obj2.b - obj2.a * obj1.b;
    if(abs(det) < 1e-3)
        px = 0;
        py = 0;
    else
        px = (obj2.b * obj1.c - obj1.b * obj2.c) / det;
        py = (obj1.a * obj2.c - obj2.a * obj1.c) / det;
    end
end