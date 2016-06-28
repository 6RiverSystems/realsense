function [r, c] = mm_world2cell(map, x, y)
    
    % Invert the row so that (0, 0) is the 
    % bottom-left corner of the map
    r = map.heightCells - mm_distance2cell(map, y) + 1;
    
    c = mm_distance2cell(map, x);
end