function cells = mm_distance2cell(map, distance)

    cells = ceil(distance / map.resolution);
    if cells == 0
        cells = 1;
    end
end