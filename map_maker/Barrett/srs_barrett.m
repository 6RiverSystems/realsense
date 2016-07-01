%
% (c) Copyright 2015-2016 River Systems, all rights reserved
%
% This is proprietary software, unauthorized distribution is not permitted.
%

map = mm_create(35, 28, 0.10);

% Full border
map = mm_addCustomBorder(map, 11.201, 1.50, 29.420, 27.137, 0.4);

% Obstacle RE02
map = mm_addStaticObstacle(map, 14.113, 5.007, 16.485, 23.759, 0.8);

% Obstacle RE03
map = mm_addStaticObstacle(map, 18.307, 5.007, 20.674, 23.759, 0.8);

% Obstacle RE04
map = mm_addStaticObstacle(map, 22.491, 5.007, 25.048, 23.759, 0.8);

% Obstacle RE05
map = mm_addStaticObstacle(map, 26.869, 5.007, 29.420, 23.759, 0.8);

% Obstacle OB01
map = mm_addStaticObstacle(map, 23.253, 3.281, 24.006, 3.535, 0.5);

map = mm_generate(map);
mm_save(map, '../../srs_sites/src/srsc_barrett/map', 'barrett');
