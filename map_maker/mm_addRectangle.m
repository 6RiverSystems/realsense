function map = mm_addRectangle(map, x1, y1, x2, y2, info)

    x1 = max(0, x1);
    y1 = max(0, y1);
    x2 = max(0, x2);
    y2 = max(0, y2);
    
    [r1, c1] = mm_world2cell(map, x1, y1);
    [r2, c2] = mm_world2cell(map, x2, y2);
    
    for r = r2:r1
        for c = c1:c2
            
            if r >= 1 && r <= map.heightCells && ...
               c >= 1 && c <= map.widthCells
                if isnumeric(info)
                    map.data(r, c).cost = info;
                else
                    map.data(r, c).(info) = true;
                end
            end
        end
    end
end