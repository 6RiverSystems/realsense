close all;
clear variables;
RESOLUTION = 0.1;
ORIGIN = [0, 0, 0];

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Main map layer
map = png2Map('/Users/fsantini/Projects/repos/ros/srs_sites/src/srsc_empty/src/empty-logical-border.png', ...
    RESOLUTION, ORIGIN);
showMap(map, 'Border');

% Make sure that the border is marked and sealed
map = convertToBorderObstacle(map, 'all', 1, 1);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Add the layers and generate the final map
showMap(map, 'Complete map');
saveGeoJsonMap(map, ...
    '/Users/fsantini/Projects/repos/ros/srs_sites/src/srsc_empty/map/empty.geojson');

open('/Users/fsantini/Projects/repos/ros/srs_sites/src/srsc_empty/map/empty.geojson');