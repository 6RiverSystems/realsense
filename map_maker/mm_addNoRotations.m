function map = mm_addNoRotations(map, x1, y1, x2, y2)

    % Add the static obstacle
    map = mm_addRectangle(map, x1, y1, x2, y2, 'NO_ROTATIONS');
end