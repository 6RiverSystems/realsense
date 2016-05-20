#include <srslib_framework/search/SearchPositionNote.hpp>

namespace srs {

const SearchPositionNote SearchPositionNote::DISABLE_OD =
    SearchPositionNote(false, false, false, true);

const SearchPositionNote SearchPositionNote::ENABLE_OD =
    SearchPositionNote(true, false, false, true);

const SearchPositionNote SearchPositionNote::GO_SLOW =
    SearchPositionNote(false, true, false, false);

const SearchPositionNote SearchPositionNote::NO_ROTATIONS =
    SearchPositionNote(false, false, true, false);

const SearchPositionNote SearchPositionNote::STATIC_OBSTACLE =
    SearchPositionNote(false, false, false, true);

}
