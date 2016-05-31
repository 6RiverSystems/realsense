function map = mm_addSetOdRectangle(map, x1, y1, x2, y2, value)
    
    % Add the static obstacle
    if value
        map = mm_addRectangle(map, x1, y1, x2, y2, 'STATIC_OBSTACLE');
    else
        map = mm_addRectangle(map, x1, y1, x2, y2, 'STATIC_OBSTACLE');
    end
    
    % Add the additional cost for the envelope
    if envelope > 0
        map = mm_addRectangle(map, ...
            x1 - envelope, y1 - envelope, ...
            x2 + envelope, y2 + envelope, ...
            cost);
    end
end