close all;
clear variables;

RESOLUTION = 0.05;
ORIGIN = [-0.025, -0.025, 0.000];

IN_IMAGE_MAIN = './hbc-logical-border.png';
IN_IMAGE_OBSTACLES = './hbc-logical-obstacles.png';

IN_IMAGE_REDUCED_SPEED = './hbc-logical-reduced_speed.png';
MAX_VELOCITY = 0.4;

IN_IMAGE_WARNING_SOUND = './hbc-logical-warning_sound.png';
IN_IMAGE_ONEWAY_WEST_EAST = './hbc-logical-one_way_west-east.png';
COST_ONEWAY_WEST_EAST = 100;

IN_IMAGE_ONEWAY_EAST_WEST = './hbc-logical-one_way_east-west.png';
COST_ONEWAY_EAST_WEST = 100;

IN_IMAGE_ONEWAY_NORTH_SOUTH = './hbc-logical-one_way_north-south.png';
COST_ONEWAY_NORTH_SOUTH = 100;

IN_IMAGE_COST_MEETING_AREA = './hbc-logical-meeting_area.png';
COST_MEETING_AREA = 10;

OUT_GEOJSON = '../map/hbc.geojson';

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Main map layer
map = png2Map(IN_IMAGE_MAIN, RESOLUTION, ORIGIN);
showMap(map, 'Border');

% Make sure that the border is marked and sealed
map = convertToBorderObstacle(map, 'all', 1, 1);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Obstacles
layerObstacles = png2Map(IN_IMAGE_OBSTACLES, RESOLUTION, ORIGIN);
showMap(layerObstacles, 'Obstacles');

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Reduced speed layer
layerReducedSpeed = png2Map(IN_IMAGE_REDUCED_SPEED, RESOLUTION, ORIGIN);
showMap(layerReducedSpeed, 'Reduced speed');

layerReducedSpeed = convertToLabeledArea(layerReducedSpeed, 'all', 'rs', ...
    struct('set_max_velocity', MAX_VELOCITY, 'play_sound', 'warning'));

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Warning sound layer
layerWarningSound = png2Map(IN_IMAGE_WARNING_SOUND, RESOLUTION, ORIGIN);
showMap(layerWarningSound, 'Warning sound');

layerWarningSound = convertToLabeledArea(layerWarningSound, 'all', 'ws', ...
    struct('play_sound', 'warning'));

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% One way West/East
layerOneWayWestEast = png2Map(IN_IMAGE_ONEWAY_WEST_EAST, RESOLUTION, ORIGIN);
showMap(layerOneWayWestEast, 'One way: allowed West-East');

layerOneWayWestEast = convertToWeightedArea(layerOneWayWestEast, 'all', ...
    0, 0, 0, COST_ONEWAY_WEST_EAST);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% One way East/West
layerOneWayEastWest = png2Map(IN_IMAGE_ONEWAY_EAST_WEST, RESOLUTION, ORIGIN);
showMap(layerOneWayEastWest, 'One way: allowed East-West');

layerOneWayEastWest = convertToWeightedArea(layerOneWayEastWest, 'all', ...
    0, COST_ONEWAY_EAST_WEST, 0, 0);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% One way North/South
layerOneWayNorthSouth = png2Map(IN_IMAGE_ONEWAY_NORTH_SOUTH, RESOLUTION, ORIGIN);
showMap(layerOneWayNorthSouth, 'One way: allowed North-South');

layerOneWayNorthSouth = convertToWeightedArea(layerOneWayNorthSouth, 'all', ...
    0, COST_ONEWAY_NORTH_SOUTH, 0, 0);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% One way North-South
layerMeeting = png2Map(IN_IMAGE_COST_MEETING_AREA, RESOLUTION, ORIGIN);
showMap(layerMeeting, 'Meeting area');

layerMeeting = convertToCostArea(layerMeeting, 'all', COST_MEETING_AREA);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Add the layers and generate the final map
map = addLayer(map, {...
    layerObstacles, ...
    layerReducedSpeed, ...
    layerWarningSound, ...
    layerOneWayWestEast, ...
    layerOneWayEastWest, ...
    layerOneWayNorthSouth,...
    layerMeeting});
showMap(map, 'Complete map');
saveGeoJsonMap(map, OUT_GEOJSON);

open(OUT_GEOJSON);
