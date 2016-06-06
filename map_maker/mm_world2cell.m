function [r, c] = mm_world2cell(map, x, y)

    % Invert the row so that (0, 0) is the 
    % bottom-left corner of the map
    r = mm_distance2cell(map, map.heightM - y);
    
    c = mm_distance2cell(map, x);
end