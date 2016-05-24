map = mm_create(10, 20, 0.1);

map = mm_addCustomBorder(map, 1.339, 1.833, 9.264, 19.521);
map = mm_addStaticObstacle(map, 3.168, 6.253, 4.387, 16.692, 0.5, 255);
map = mm_addStaticObstacle(map, 6.216, 3.662, 7.435, 16.692, 0.5, 255);

map = mm_generate(map);
mm_save(map, '../../srs_sites/src/srsc_6rhq/map', '6rhq');