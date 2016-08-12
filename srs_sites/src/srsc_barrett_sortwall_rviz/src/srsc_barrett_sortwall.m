%
% (c) Copyright 2015-2016 River Systems, all rights reserved
%
% This is proprietary software, unauthorized distribution is not permitted.
%

map = mm_create(37, 92, 0.10);

OBSTACLE = 1.25;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Full border

% Edge
map = mm_addCustomBorder(map, 0.0, 0.0, 36.36, 91.23, 0.4);

% OB00
map = mm_addStaticObstacle(map, 0.0, 0.0, 13.23, 59.67, 0.7);

% OB01
map = mm_addStaticObstacle(map, 0.0, 59.67, 13.79, 86.40 + OBSTACLE, 0.7);

% OB02
map = mm_addStaticObstacle(map, 13.23, 0.0, 28.90, 1.52, 0.7);

% OB03
map = mm_addStaticObstacle(map, 25.35, 3.21, 26.25, 3.61, 0.6);

% OB04
map = mm_addStaticObstacle(map, 28.90, 0.0, 36.36, 23.76 + OBSTACLE, 0.7);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Racks

% RE01
%map = mm_addStaticObstacle(map, 16.15, 27.14, 27.71, 56.37, 0.7);

% PACK01
map = mm_addStaticObstacle(map, 16.15, 27.14, 27.71, 38.59, 0.7);

% PACK02
map = mm_addStaticObstacle(map, 19.81, 38.59, 27.71, 49.05, 0.3);

% PACK03
map = mm_addStaticObstacle(map, 16.15, 49.05, 27.71, 56.37, 0.7);

% RE02
map = mm_addStaticObstacle(map, 16.15, 5.01 - OBSTACLE, 18.52, 23.76 + OBSTACLE, 0.7);

% RE03
map = mm_addStaticObstacle(map, 20.34, 5.01 - OBSTACLE, 22.71, 23.76 + OBSTACLE, 0.7);

% RE04
map = mm_addStaticObstacle(map, 24.52, 5.01, 27.08, 23.76 + OBSTACLE, 0.7);

% RAB1
map = mm_addStaticObstacle(map, 30.07, 27.14, 32.04, 41.06, 0.7);

% RAB2
map = mm_addStaticObstacle(map, 30.07, 43.47, 32.04, 56.37, 0.7);

% RCD
map = mm_addStaticObstacle(map, 34.39, 27.14, 36.36, 56.37, 0.7);

% RH01
map = mm_addStaticObstacle(map, 16.79, 59.67 - OBSTACLE, 20.40, 86.40 + OBSTACLE, 0.7);
 
% RG01
map = mm_addStaticObstacle(map, 23.42, 59.67 - OBSTACLE, 28.42, 86.40 + OBSTACLE, 0.7);
 
% RF01
map = mm_addStaticObstacle(map, 31.39, 59.67 - OBSTACLE, 36.36, 86.40 + OBSTACLE, 0.7);
 
% RI01
map = mm_addStaticObstacle(map, 0.0, 89.27, 36.36, 91.23, 0.7);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Sort Wall Stalls

% ST01
map = mm_addStaticObstacle(map, 16.55, 41.47, 17.00, 43.57, 0.2);

% ST02
map = mm_addStaticObstacle(map, 16.55, 44.72, 17.00, 44.82, 0.2);

% ST03
map = mm_addStaticObstacle(map, 16.55, 45.97, 17.00, 46.07, 0.2);

% ST04
map = mm_addStaticObstacle(map, 16.55, 47.22, 17.00, 47.32, 0.2);

map = mm_generate(map);
mm_save(map, '../../srs_sites/src/srsc_barrett_sortwall/map', 'barrett');
mm_save(map, '../../srs_sites/src/srsc_barrett_sortwall_rviz/map', 'barrett');