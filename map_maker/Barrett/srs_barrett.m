%
% (c) Copyright 2015-2016 River Systems, all rights reserved
%
% This is proprietary software, unauthorized distribution is not permitted.
%

map = mm_create(25, 28, 0.10);

% Full border
map = mm_addCustomBorder(map, 0.00, 0.00, 21.16, 26.92, 0.4);

% Obstacle A
map = mm_addStaticObstacle(map, 0.00, 0.00, 21.16, 1.52, 0.4);

% Obstacle B
map = mm_addStaticObstacle(map, 12.07, 3.10, 12.88, 3.35, 0.5);

% Obstacle C
map = mm_addStaticObstacle(map, 2.92, 4.80, 5.28, 23.55, 0.5);

% Obstacle D
map = mm_addStaticObstacle(map, 7.11, 4.80, 9.47, 23.55, 0.5);

% Obstacle E
map = mm_addStaticObstacle(map, 11.30, 4.80, 13.67, 23.55, 0.5);

% Obstacle F
map = mm_addStaticObstacle(map, 15.67, 4.80, 18.03, 23.55, 0.5);

% Obstacle G
map = mm_addStaticObstacle(map, 19.76, 4.80, 21.16, 23.55, 0.5);

map = mm_generate(map);
mm_save(map, '../../srs_sites/src/srsc_barrett/map', 'barrett');
