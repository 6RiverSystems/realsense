map = mm_create(20, 10, 0.1);

map = mm_addCustomBorder(map, 1.833, 1.339, 19.521, 9.264, 0.5);
map = mm_addStaticObstacle(map, 6.253, 3.168, 16.692, 4.387);
map = mm_addStaticObstacle(map, 3.662, 6.216, 16.692, 7.435);

map = mm_generate(map);
mm_save(map, '../../srs_sites/src/srsc_6rhq/map', '6rhq');
mm_save(map, '../../srs_sites/src/srsc_6rhq_norviz/map', '6rhq');