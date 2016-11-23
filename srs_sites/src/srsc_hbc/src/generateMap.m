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
layerOneWayWestEast = png2Map('/Users/fsantini/Projects/repos/ros/srs_sites/src/srsc_hbc/src/hbc-logical-one_way_west-east.png', ...
    RESOLUTION, ORIGIN);
showMap(layerOneWayWestEast, 'West weight');

layerOneWayWestEast = convertToWeightedArea(layerOneWayWestEast, 'all', 0, 0, 0, 100);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% One way East
layerOneWayEastWest = png2Map('/Users/fsantini/Projects/repos/ros/srs_sites/src/srsc_hbc/src/hbc-logical-one_way_east-west.png', ...
    RESOLUTION, ORIGIN);
showMap(layerOneWayEastWest, 'East weight');

layerOneWayEastWest = convertToWeightedArea(layerOneWayEastWest, 'all', 0, 100, 0, 0);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% One way North-South
layerMeeting = png2Map('/Users/fsantini/Projects/repos/ros/srs_sites/src/srsc_hbc/src/hbc-logical-one_way_meeting.png', ...
    RESOLUTION, ORIGIN);
showMap(layerMeeting, 'Meeting area');

layerMeeting = convertToCostArea(layerMeeting, 'all', 100);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Add the layers and generate the final map
map = addLayer(map, {layerWarningSound, layerOneWayWestEast, layerOneWayEastWest, layerMeeting});
showMap(map, 'Total map');

saveGeoJsonMap(map, ...
    '/Users/fsantini/Projects/repos/ros/srs_sites/src/srsc_hbc/map/hbc.geojson');
open('/Users/fsantini/Projects/repos/ros/srs_sites/src/srsc_hbc/map/hbc.geojson');
