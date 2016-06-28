function result = mm_quantize(map, value)

    result = map.resolution * floor(value / map.resolution);
end