#include <srslib_framework/search/SearchPositionNote.hpp>

namespace srs {

const SearchPositionNote SearchPositionNote::STATIC_OBSTACLE = SearchPositionNote(false, false, true);
const SearchPositionNote SearchPositionNote::GO_SLOW = SearchPositionNote(true, false, false);
const SearchPositionNote SearchPositionNote::NO_ROTATIONS = SearchPositionNote(false, true, false);

}
