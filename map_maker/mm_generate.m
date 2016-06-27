function map = mm_generate(map)

    RGB_RED = 1;
    RGB_GREEN = 2;
    RGB_BLUE = 3;
    
    BIT_OD = 8;
    BIT_GO_SLOW = 7;
    BIT_NO_ROTATIONS = 6;

    MAX_VALUE = intmax('uint8');
    
    for r = 1:map.heightCells
        for c = 1:map.widthCells
            
            data = map.data(r, c);
            
            % The default pixels has:
            % - no static-obstacle
            % - OD is enabled
            % - general cost is 0
            pixel = uint8([0, 128, 0]);
            
            cost = data.cost;
            if isempty(cost)
                cost = 0;
            end
            if cost > MAX_VALUE
                cost = MAX_VALUE;
            end
            
            if checkFlag(data, 'STATIC_OBSTACLE')
                pixel(RGB_RED) = MAX_VALUE;
                cost = 0;
            end
            if checkFlag(data, 'OD')
                bitset(pixel(RGB_GREEN), BIT_OD, data.('OD'));
            end
            if checkFlag(data, 'GO_SLOW')
                bitset(pixel(RGB_GREEN), BIT_GO_SLOW, true);
            end
            if checkFlag(data, 'NO_ROTATIONS')
                bitset(pixel(RGB_GREEN), BIT_NO_ROTATIONS, true);
            end
            
            pixel(RGB_BLUE) = uint8(cost);
            
            map.image(r, c, 1) = pixel(RGB_RED);
            map.image(r, c, 2) = pixel(RGB_GREEN);
            map.image(r, c, 3) = pixel(RGB_BLUE);
        end
    end
    
    imshow(map.image);
end

function flag = checkFlag(data, field)

    flag = false;
    if isfield(data, field)
        if ~isempty(data.(field)) 
            flag = data.(field);
        end
    end 
end