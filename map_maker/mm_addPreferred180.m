function map = mm_addPreferred180(map, x1, y1, x2, y2)

    % Add the static obstacle
    map = mm_addRectangle(map, x1, y1, x2, y2, 'PREFERRED_180');
end