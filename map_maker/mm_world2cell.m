function [r, c] = mm_world2cell(map, x, y)

    r = mm_distance2cell(map, y);
    c = mm_distance2cell(map, x);
end