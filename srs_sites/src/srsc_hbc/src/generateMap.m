close all;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Main map layer
map = png2Map('/Users/fsantini/Projects/repos/ros/srs_sites/src/srsc_hbc/src/hbc-logical-obstacles.png');
showMap(map);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Honk layer
layerHonk = png2Map('/Users/fsantini/Projects/repos/ros/srs_sites/src/srsc_hbc/src/hbc-logical-honk.png');
showMap(layerHonk);

layerHonk = convertToLabeledArea(layerHonk, 1, 'honk1', {'honk'});
layerHonk = convertToLabeledArea(layerHonk, 2, 'honk2', {'honk'});
layerHonk = convertToLabeledArea(layerHonk, 3, 'honk3', {'honk'});
layerHonk = convertToLabeledArea(layerHonk, 4, 'honk4', {'honk'});

% Add the layers and generate the final map
map = addLayer(map, {layerHonk, layerHonk});
showMap(map);

saveGeoJsonMap(map, ...
    '/Users/fsantini/Projects/repos/ros/srs_sites/src/srsc_hbc/map/6rshq_honk.geojson');
open('/Users/fsantini/Projects/repos/ros/srs_sites/src/srsc_hbc/map/6rshq_honk.geojson');
