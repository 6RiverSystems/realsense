function cells = mm_distance2cell(map, distance)

    resolution = map.resolution;
    w = resolution * 0.1;
    k = (abs(distance) - w / 2) / resolution + 1;
    cells = floor(max(0, k));
    
    if cells == 0
        cells = 1;
    end
end