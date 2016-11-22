close all;
RESOLUTION = 0.05;
ORIGIN = [-0.025, -0.025, 0.000];

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Main map layer
map = png2Map('/Users/fsantini/Projects/repos/ros/srs_sites/src/srsc_hbc/src/hbc-logical-obstacles.png', ...
    RESOLUTION, ORIGIN);
showMap(map, 'Obstacles');

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Warning sound layer
layerWarningSound = png2Map('/Users/fsantini/Projects/repos/ros/srs_sites/src/srsc_hbc/src/hbc-logical-warning_sound.png', ...
    RESOLUTION, ORIGIN);
showMap(layerWarningSound, 'Warning sound');

layerWarningSound = convertToLabeledArea(layerWarningSound, 'all', 'ws', {'warning_sound'});

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% One way West
layerOneWayWest = png2Map('/Users/fsantini/Projects/repos/ros/srs_sites/src/srsc_hbc/src/hbc-logical-one_way_west.png', ...
    RESOLUTION, ORIGIN);
showMap(layerOneWayWest, 'West weight');

layerOneWayWest = convertToWeightedArea(layerOneWayWest, 'all', 0, 0, 0, 100);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% One way East
layerOneWayEast = png2Map('/Users/fsantini/Projects/repos/ros/srs_sites/src/srsc_hbc/src/hbc-logical-one_way_east.png', ...
    RESOLUTION, ORIGIN);
showMap(layerOneWayEast, 'East weight');

layerOneWayEast = convertToWeightedArea(layerOneWayEast, 'all', 0, 100, 0, 0);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% One way North-South
layerOneWayNorthSouth = png2Map('/Users/fsantini/Projects/repos/ros/srs_sites/src/srsc_hbc/src/hbc-logical-one_way_north_south.png', ...
    RESOLUTION, ORIGIN);
showMap(layerOneWayNorthSouth, 'North-South weight');

layerOneWayNorthSouth = convertToWeightedArea(layerOneWayNorthSouth, 'all', 10, 0, 10, 0);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Add the layers and generate the final map
map = addLayer(map, {layerWarningSound, layerOneWayWest, layerOneWayEast, layerOneWayNorthSouth});
showMap(map, 'Total map');

saveGeoJsonMap(map, ...
    '/Users/fsantini/Projects/repos/ros/srs_sites/src/srsc_hbc/map/hbc.geojson');
open('/Users/fsantini/Projects/repos/ros/srs_sites/src/srsc_hbc/map/hbc.geojson');
