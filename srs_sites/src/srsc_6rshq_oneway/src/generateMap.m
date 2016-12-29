close all;
clear variables;

RESOLUTION = 0.05;
ORIGIN = [-10, -10, 0];

INPUT_DIR = '/Users/fsantini/Projects/repos/ros/srs_sites/src/srsc_6rshq_oneway/src/';
OUTPUT_DIR = '/Users/fsantini/Projects/repos/ros/srs_sites/src/srsc_6rshq_oneway/map/';

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Main map layer
map = png2Map(sprintf('%s%s', INPUT_DIR, '6rshq_oneway-logical-border.png'), ...
    RESOLUTION, ORIGIN);
showMap(map, 'Obstacles');

% Make sure that the border is marked and sealed
map = convertToBorderObstacle(map, 'all', 1, 1);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Obstacles
layerObstacles = png2Map(sprintf('%s%s', INPUT_DIR, '6rshq_oneway-logical-obstacles.png'), ...
    RESOLUTION, ORIGIN);
showMap(layerObstacles, 'Obstacles');

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% One-way layer
layerOneway = png2Map(sprintf('%s%s', INPUT_DIR, '6rshq_oneway-logical-oneway.png'), ...
    RESOLUTION, ORIGIN);
showMap(layerOneway, 'One-ways');

layerOneway = convertToWeightedArea(layerOneway, 1, 0, 0, 0, 100);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Warning sound layer
layerWarningSound = png2Map(sprintf('%s%s', INPUT_DIR, '6rshq_oneway-logical-warning_sound.png'), ...
    RESOLUTION, ORIGIN);
showMap(layerWarningSound, 'Warning sound');

layerWarningSound = convertToLabeledArea(layerWarningSound, 'all', 'ws', ...
        struct('play_sound', 'warning', 'set_max_velocity', 0.3));

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Reduced speed layer
layerReducedSpeed = png2Map('/Users/fsantini/Projects/repos/ros/srs_sites/src/srsc_6rshq/src/6rshq-logical-reduced_speed.png', ...
    RESOLUTION, ORIGIN);
showMap(layerReducedSpeed, 'Reduced speed');

layerReducedSpeed = convertToLabeledArea(layerReducedSpeed, 'all', 'ws', ...
    struct('set_max_velocity', 0.4));

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Graph layer

% AA-------------------------------------BB
% |                                       |
% |                                       |
% A --------------------------------------B
% |                                       |
% |     *****************************     |
% |     *****************************     |
% |                                       |
% C-(P2)D------------------------------(P1)-E
% |       |                               |
% |       |  ************************     |
% |       |  ************************     |
% |       |                  P3           |
% F-------G-------------------------------H
% |                                       |
% |                                       |
% |                                       |
% FF-------------------------------------HH

map = addVertex(map, 'A', 'xy', [2.748 8.349]);
map = addVertex(map, 'B', 'xy', [17.606 8.349]);
map = addVertex(map, 'C', 'xy', [2.748 5.302]);
map = addVertex(map, 'D', 'xy', [5.339 5.302]);
map = addVertex(map, 'E', 'xy', [17.606 5.302]);
map = addVertex(map, 'F', 'xy', [2.748 2.254]);
map = addVertex(map, 'G', 'xy', [5.339 2.254]);
map = addVertex(map, 'H', 'xy', [17.606 2.254]);
map = addVertex(map, 'AA', 'xy', [2.748 8.864]);
map = addVertex(map, 'BB', 'xy', [17.606 8.864]);
map = addVertex(map, 'FF', 'xy', [2.748 1.600]);
map = addVertex(map, 'HH', 'xy', [17.606 1.600]);

map = addVertex(map, 'rockyInduct', 'xy', [9.748 8.664], ...
    struct('orientation', 0, 'labels', {'induct'}, ...
    'mfp_ids', {'rocky'}));

map = addVertex(map, 'borisInduct', 'xy', [5.000 2.254], ...
    struct('orientation', 0, 'labels', {'induct'}, ...
    'mfp_ids', {'boris'}));

map = addVertex(map, 'bullInduct', 'xy', [13.748 8.664], ...
    struct('orientation', 0, 'labels', {'induct'}, ...
    'mfp_ids', {'bullwinkle'}));

map = addVertex(map, 'picking1', 'xy', [17.606 5.302], ...
    struct('orientation', 180.0, 'labels', {'picking'}, ...
    'mfp_ids', {'rocky'}));

map = addVertex(map, 'picking2', 'xy', [2.748 5.302], ...
    struct('orientation', 0, 'labels', {'picking'}));

map = addVertex(map, 'picking3', 'xy', [14.600 2.100], ...
    struct('orientation', 180.0, 'labels', {'picking'}));

map = addVertex(map, 'rockyPackout', 'xy', [3.200 10.000], ...
    struct('orientation', 270.0, 'labels', {'packout'}, ...
    'mfp_ids', {'rocky'}));

map = addVertex(map, 'borisPackout', 'xy', [2.000 3.000], ...
    struct('orientation', 270.0, 'labels', {'packout'}, ...
    'mfp_ids', {'boris'}));

map = addVertex(map, 'bullPackout', 'xy', [5.600 10.000], ...
    struct('orientation', 270.0, 'labels', {'packout'}, ...
    'mfp_ids', {'bullwinkle'}));

map = addVertex(map, 'lblInduct', 'xy', [2.200 3.200], ...
    struct('orientation', 90.0, 'labels', {'induct'}, ...
    'mfp_ids', {'leah', 'ben', 'leo', 'lily', 'caroline', ...
    'elle', 'ava', 'javier', 'alice', 'sam', 'dan', 'max', 'ripley'}));

map = addVertex(map, 'ahInduct', 'xy', [2.200 3.200], ...
    struct('orientation', 90.0, 'labels', {'induct'}, ...
    'mfp_ids', {'arroway','harry'}));

map = addVertex(map, 'lblPackout', 'xy', [5.000 2.254], ...
    struct('orientation', 180.0, 'labels', {'packout'}, ...
    'mfp_ids', {'leah', 'ben', 'leo', 'lily', 'caroline', 'elle', ...
    'ava', 'javier', 'alice', 'sam', 'dan', 'max', 'ripley'}));

map = addVertex(map, 'ahPackout', 'xy', [5.000 2.254], ...
    struct('orientation', 180.0, 'labels', {'packout'}, ...
    'mfp_ids', {'arroway','harry'}));

map = addEdge(map, 'AB', 'A', 'B', 'none');
map = addEdge(map, 'CE', 'C', 'E', 'none');
map = addEdge(map, 'FH', 'F', 'H', 'none');
map = addEdge(map, 'AABB', 'AA', 'BB', 'none');
map = addEdge(map, 'FFHH', 'FF', 'HH', 'none');

map = addGraph(map, '6rshq_graph', ...
    {'A', 'B', 'C', 'D', 'E', 'F', 'G', 'H', 'AA', 'BB', 'FF', 'HH', ...
    'rockyInduct', 'borisInduct', 'bullInduct', 'picking1', ...
    'picking2', 'picking3', 'rockyPackout', 'borisPackout', ...
    'bullPackout', 'lblInduct', 'ahInduct', 'lblPackout', 'ahPackout'}, ...
    {'AB', 'CE', 'FH', 'AABB', 'FFHH'});

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Add the layers and generate the final map
map = addLayer(map, {...
    layerObstacles, ...
    layerWarningSound, ...
    layerOneway});
showMap(map, 'Complete map');
saveGeoJsonMap(map, sprintf('%s%s', OUTPUT_DIR, '6rshq_oneway.geojson'));

open(sprintf('%s%s', OUTPUT_DIR, '6rshq_oneway.geojson'));