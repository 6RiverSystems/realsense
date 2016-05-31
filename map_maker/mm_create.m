function map = mm_create(width, height, resolution)

    map.widthM = width;
    map.heightM = height;
    map.resolution = resolution;
    
    map.widthCells = ceil(width / resolution);
    map.heightCells = ceil(height / resolution);
    map.channels = 3;

    map.data(map.heightCells, map.widthCells).cost = 0;
    
    map.image = uint8(zeros(map.heightCells, map.widthCells, map.channels));
end