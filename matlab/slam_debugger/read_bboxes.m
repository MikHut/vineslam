function bboxes = read_bboxes(file, type)
    data = textread(file, '%s');
    
    k = 1;
    j = 1;
    l = 1;
    while(1)
        if data(l) == "trunk"
            if type == "dets"
                l = l+1;
            end
            j = j+1;
            k = 1;
        else
            bboxes(j-1,k) = data(l);
            k = k + 1;
        end
        l = l+1;
        if l > size(data)
            break;
        end
    end
end