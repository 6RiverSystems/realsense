%
% (c) Copyright 2015-2016 River Systems, all rights reserved
%
% This is proprietary software, unauthorized distribution is not permitted.
%

map = mm_create(20, 13, 0.1);

% map = mm_addCustomBorder(map, 1.833, 1.339, 19.521, 9.264, 0.4);
% tweaking size of white space to make room for sort-wall
map = mm_addCustomBorder(map, 1.833, 1.339, 19.521, 12.264, 0.4);

% Obstacle Rack 1-2
map = mm_addStaticObstacle(map, 6.253, 3.168, 16.692, 4.387, 0.7);

% Obstacle Rack 3-4
map = mm_addStaticObstacle(map, 3.662, 6.216, 16.692, 7.435, 0.7);

% Obstacle Workshop
map = mm_addStaticObstacle(map, 9.148, 9.264, 19.521, 12.264, 0.7);

% Obstacle Stall 1
map = mm_addStaticObstacle(map, 3.662, 8.664, 3.762, 9.664, 0.3);

% Obstacle Stall 2
map = mm_addStaticObstacle(map, 4.912, 8.664, 5.012, 9.664, 0.3);

map = mm_generate(map);
mm_save(map, '../../srs_sites/src/srsc_6rhq/map', '6rhq_sortwall');
mm_save(map, '../../srs_sites/src/srsc_6rhq_rviz/map', '6rhq_sortwall');