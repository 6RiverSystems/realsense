map = mm_create(10, 18, 0.1);

map = mm_addBorder(map, 0.5);
map = mm_addStaticObstacle(map, 6, 6, 6.2, 6.2, 0.5, 255);

map = mm_generate(map);
mm_save(map, '.', '6rhq');