%
% (c) Copyright 2015-2016 River Systems, all rights reserved
%
% This is proprietary software, unauthorized distribution is not permitted.
%

map = mm_create(20, 10, 0.1);

map = mm_addCustomBorder(map, 1.833, 1.339, 19.521, 9.264, 0.4);

% Obstacle A
map = mm_addStaticObstacle(map, 6.253, 3.168, 16.692, 4.387, 0.7);

% 180deg orientation in the middle aisle
map = mm_addPreferred180(map, 6.253, 4.5, 16.692, 6.2);

% Obstacle B
map = mm_addStaticObstacle(map, 3.662, 6.216, 16.692, 7.435, 0.7);

map = mm_generate(map);
mm_save(map, '../map', 'demo');
