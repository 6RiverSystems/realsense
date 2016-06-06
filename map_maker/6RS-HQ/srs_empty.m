map = mm_create(20, 10, 0.1);

map = mm_addCustomBorder(map, 1.833, 1.339, 19.521, 9.264, 0.5);

map = mm_generate(map);
mm_save(map, '../../srs_sites/src/srsc_empty/map', 'empty');
mm_save(map, '../../srs_sites/src/srsc_empty_norviz/map', 'empty');