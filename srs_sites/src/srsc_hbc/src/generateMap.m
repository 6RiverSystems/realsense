close all;
clear variables;
RESOLUTION = 0.05;
ORIGIN = [-0.025, -0.025, 0.000];

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Main map layer
map = png2Map('/Users/fsantini/Projects/repos/ros/srs_sites/src/srsc_hbc/src/hbc-logical-border.png', ...
    RESOLUTION, ORIGIN);
showMap(map, 'Border');

% Make sure that the border is marked and sealed
map = convertToBorderObstacle(map, 'all', 1, 1);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Obstacles
layerObstacles = png2Map('/Users/fsantini/Projects/repos/ros/srs_sites/src/srsc_hbc/src/hbc-logical-obstacles.png', ...
    RESOLUTION, ORIGIN);
showMap(layerObstacles, 'Obstacles');

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Reduced speed layer
layerReducedSpeed = png2Map('/Users/fsantini/Projects/repos/ros/srs_sites/src/srsc_hbc/src/hbc-logical-reduced_speed.png', ...
    RESOLUTION, ORIGIN);
showMap(layerReducedSpeed, 'Reduced speed');

layerReducedSpeed = convertToLabeledArea(layerReducedSpeed, 'all', 'ws', ...
    struct('set_max_velocity', 0.4));

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Warning sound layer
layerWarningSound = png2Map('/Users/fsantini/Projects/repos/ros/srs_sites/src/srsc_hbc/src/hbc-logical-warning_sound.png', ...
    RESOLUTION, ORIGIN);
showMap(layerWarningSound, 'Warning sound');

layerWarningSound = convertToLabeledArea(layerWarningSound, 'all', 'ws', ...
    struct('play_sound', 'warning'));

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% One way West
layerOneWayWestEast = png2Map('/Users/fsantini/Projects/repos/ros/srs_sites/src/srsc_hbc/src/hbc-logical-one_way_west-east.png', ...
    RESOLUTION, ORIGIN);
showMap(layerOneWayWestEast, 'One way: allowed West-East');

layerOneWayWestEast = convertToWeightedArea(layerOneWayWestEast, 'all', 0, 0, 0, 100);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% One way East
layerOneWayEastWest = png2Map('/Users/fsantini/Projects/repos/ros/srs_sites/src/srsc_hbc/src/hbc-logical-one_way_east-west.png', ...
    RESOLUTION, ORIGIN);
showMap(layerOneWayEastWest, 'One way: allowed East-West');

layerOneWayEastWest = convertToWeightedArea(layerOneWayEastWest, 'all', 0, 100, 0, 0);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% One way North-South
layerMeeting = png2Map('/Users/fsantini/Projects/repos/ros/srs_sites/src/srsc_hbc/src/hbc-logical-meeting_area.png', ...
    RESOLUTION, ORIGIN);
showMap(layerMeeting, 'Meeting area');

layerMeeting = convertToCostArea(layerMeeting, 'all', 50);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Add the layers and generate the final map
map = addLayer(map, {...
    layerObstacles, ...
    layerReducedSpeed, ...
    layerWarningSound, ...
    layerOneWayWestEast, ...
    layerOneWayEastWest, ...
    layerMeeting});
showMap(map, 'Total map');

saveGeoJsonMap(map, ...
    '/Users/fsantini/Projects/repos/ros/srs_sites/src/srsc_hbc/map/hbc.geojson');
open('/Users/fsantini/Projects/repos/ros/srs_sites/src/srsc_hbc/map/hbc.geojson');
