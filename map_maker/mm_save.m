function mm_save(map, outputDirectory, mapName)

    yaml = {
        '#\n', ...
        '# (c) Copyright 2015-2016 River Systems, all rights reserved\n', ...
        '#\n', ...
        '# This is proprietary software, unauthorized distribution is not permitted.\n', ...
        '#\n', ...
        '\n', ...
        sprintf('image: %s.png\n', mapName), ...
        sprintf('resolution: %.3f\n', map.resolution) ...
    };

    fileId = fopen(sprintf('%s/%s.yaml', outputDirectory, mapName), 'w');
    for l = 1:length(yaml)
        fprintf(fileId, yaml{l});
    end
    fclose(fileId);
    
    imwrite(map.image, sprintf('%s/%s.png', outputDirectory, mapName));
end