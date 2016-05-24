function map = mm_addOuterBorder(map, thickness)

    % Top border
    map = mm_addStaticObstacle(map, ...
        0, 0, ...
        map.widthM, thickness, ...
        0.5, 255);
    
    % Bottom border
    map = mm_addStaticObstacle(map, ...
        0, map.heightM - thickness, ...
        map.widthM, map.heightM, ...
        0.5, 255);
    
    % Left border
    map = mm_addStaticObstacle(map, ...
        0, 0, ...
        thickness, map.heightM, ...
        0.5, 255);
    
    % Right border
    map = mm_addStaticObstacle(map, ...
        map.widthM - thickness, 0, ...
        map.widthM, map.heightM, ...
        0.5, 255);
end