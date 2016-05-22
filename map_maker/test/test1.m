map = mm_create(10, 18, 0.1);

map = mm_addBorder(map, 0.5);
map = addStaticObstacle(map, 0, 0, 0, 0, 0.5, 255);

map = mm_generate(map);
mm_save(map, '.', 'test');