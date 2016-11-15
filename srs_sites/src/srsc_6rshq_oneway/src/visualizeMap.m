close all;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Main map layer
map = png2Map('/Users/fsantini/Projects/repos/ros/srs_sites/src/srsc_6rshq_oneway/src/6rshq_oneway-logical-obstacles.png');
showMap(map);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% One-way layer
layerOneway = png2Map('/Users/fsantini/Projects/repos/ros/srs_sites/src/srsc_6rshq_oneway/src/6rshq_oneway-logical-oneway.png');
showMap(layerOneway);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Honk layer
layerHonk = png2Map('/Users/fsantini/Projects/repos/ros/srs_sites/src/srsc_6rshq_oneway/src/6rshq_oneway-logical-honk.png');
showMap(layerHonk);
