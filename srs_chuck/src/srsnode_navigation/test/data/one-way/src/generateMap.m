close all;
clear variables;
RESOLUTION = 1;
ORIGIN = [0.000, 0.000, 0.000];

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Main map layer
map = png2Map('/Users/fsantini/Projects/repos/ros/srs_chuck/src/srsnode_navigation/test/data/one-way/src/one-way-logical-obstacles.png', ...
    RESOLUTION, ORIGIN);
showMap(map, 'Obstacles');

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% One way West
layerOneWayWestEast = png2Map('/Users/fsantini/Projects/repos/ros/srs_chuck/src/srsnode_navigation/test/data/one-way/src/one-way-logical-west-east.png', ...
    RESOLUTION, ORIGIN);
showMap(layerOneWayWestEast, 'West weight');

layerOneWayWestEast = convertToWeightedArea(layerOneWayWestEast, 'all', 0, 0, 0, 100);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Add the layers and generate the final map
map = addLayer(map, {layerOneWayWestEast});
showMap(map, 'Complete map');
saveGeoJsonMap(map, ...
    '/Users/fsantini/Projects/repos/ros/srs_chuck/src/srsnode_navigation/test/data/one-way/one-way.geojson');

open('/Users/fsantini/Projects/repos/ros/srs_chuck/src/srsnode_navigation/test/data/one-way/one-way.geojson');
