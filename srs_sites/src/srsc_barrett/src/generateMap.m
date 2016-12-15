close all;
clear variables;
RESOLUTION = 0.1;
ORIGIN = [0.000, 0.000, 0.000];

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Main map layer
map = png2Map('/Users/fsantini/Projects/repos/ros/srs_sites/src/srsc_barrett/src/barrett-logical-border.png', ...
    RESOLUTION, ORIGIN);
showMap(map, 'Border');

map = convertToBorderObstacle(map, 'all', 1, 1);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Obstacles
layerObstacles = png2Map('/Users/fsantini/Projects/repos/ros/srs_sites/src/srsc_barrett/src/barrett-logical-obstacles.png', ...
    RESOLUTION, ORIGIN);
showMap(layerObstacles, 'Obstacles');

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Add the layers and generate the final map
map = addLayer(map, {layerObstacles});
showMap(map, 'Complete map');
saveGeoJsonMap(map, ...
    '/Users/fsantini/Projects/repos/ros/srs_sites/src/srsc_barrett/map/barrett.geojson');

open('/Users/fsantini/Projects/repos/ros/srs_sites/src/srsc_barrett/map/barrett.geojson');
