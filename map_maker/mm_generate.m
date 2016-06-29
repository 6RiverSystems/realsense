function map = mm_generate(map)

    MAX_VALUE = intmax('uint8');
    
    for r = 1:map.heightCells
        for c = 1:map.widthCells
            
            data = map.data(r, c);
            
            % The default pixels has:
            % - no static-obstacle
            % - OD is enabled
            % - general cost is 0
            red = uint8(0);
            
            cost = data.cost;
            if isempty(cost)
                cost = 0;
            end
            if cost > MAX_VALUE
                cost = MAX_VALUE;
            end
            
            if checkFlag(data, 'STATIC_OBSTACLE')
                red = MAX_VALUE;
                cost = 0;
            end
            green = calculateGreen(data);
            blue = uint8(cost);
            
            map.image(r, c, 1) = red;
            map.image(r, c, 2) = green;
            map.image(r, c, 3) = blue;
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

function green = calculateGreen(data)

    BIT_OD = 8;
    BIT_GO_SLOW = 7;
    BIT_NO_ROTATIONS = 6;
    
    BIT_PREFERRED_2 = 5;
    BIT_PREFERRED_1 = 4;
    BIT_PREFERRED_0 = 3;

    green = bitset(0, BIT_OD, true);
    
    if checkFlag(data, 'OD')
        green = bitset(green, BIT_OD, data.('OD'));
    end
    if checkFlag(data, 'GO_SLOW')
        green = bitset(green, BIT_GO_SLOW, true);
    end
    if checkFlag(data, 'NO_ROTATIONS')
        green = bitset(green, BIT_NO_ROTATIONS, true);
    end
    if checkFlag(data, 'PREFERRED_180')
        green = bitset(green, BIT_PREFERRED_2, true);
        green = bitset(green, BIT_PREFERRED_1, true);
        green = bitset(green, BIT_PREFERRED_0, false);
    end
    if checkFlag(data, 'PREFERRED_0')
        green = bitset(green, BIT_PREFERRED_2, true);
        green = bitset(green, BIT_PREFERRED_1, false);
        green = bitset(green, BIT_PREFERRED_0, false);
    end
end