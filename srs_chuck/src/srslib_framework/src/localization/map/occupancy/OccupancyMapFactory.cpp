#include <srslib_framework/localization/map/occupancy/OccupancyMapFactory.hpp>

#include <limits>

#include <yaml-cpp/yaml.h>

#include <srslib_framework/exception/io/FailedToOpenFileException.hpp>
#include <srslib_framework/localization/map/occupancy/exception/InvalidChannelNumberException.hpp>
#include <srslib_framework/localization/map/MapNote.hpp>

namespace srs {

////////////////////////////////////////////////////////////////////////////////////////////////////
// Public methods

////////////////////////////////////////////////////////////////////////////////////////////////////
OccupancyMap* OccupancyMapFactory::fromCostMap2D(costmap_2d::Costmap2D* costMap,
    double freeThreshold, double occupiedThreshold)
{
    map_ = nullptr;

    if (!costMap)
    {
        return nullptr;
    }

    unsigned int rows = costMap->getSizeInCellsY();
    unsigned int columns = costMap->getSizeInCellsX();

    map_ = new OccupancyMap(columns, rows, costMap->getResolution());
    metadata_ = map_->getMetadata();

    for (int row = 0; row < rows; row++)
    {
        for (int col = 0; col < columns; col++)
        {
            map_->setCost(col, row, static_cast<unsigned int>(costMap->getCost(col, row)));
        }
    }

    return map_;
}

////////////////////////////////////////////////////////////////////////////////////////////////////
OccupancyMap* OccupancyMapFactory::fromGrid2d(Grid2d* grid, double resolution)
{
    map_ = new OccupancyMap(grid, resolution);
    metadata_ = map_->getMetadata();

    return map_;
}

////////////////////////////////////////////////////////////////////////////////////////////////////
OccupancyMap* OccupancyMapFactory::fromMetadata(OccupancyMetadata metadata)
{
    map_ = nullptr;
    metadata_ = metadata;

    SDL_Surface* image = IMG_Load(metadata.occupancyFilename.c_str());
    if (!image)
    {
        throw FailedToOpenFileException(metadata.occupancyFilename);
    }

    // Copy the image data into the map structure
    metadata_.widthCells = image->w;
    metadata_.heightCells = image->h;

    map_ = new OccupancyMap(metadata_);

    int channels = image->format->BytesPerPixel;
    switch (channels)
    {
        case 1:
            extract1Channel(image);
            break;

        case 3:
            extract3Channel(image);
            break;

        default:
            throw InvalidChannelNumberException(metadata_.occupancyFilename, channels);
    }

    SDL_FreeSurface(image);

    return map_;
}

////////////////////////////////////////////////////////////////////////////////////////////////////
// Private methods

////////////////////////////////////////////////////////////////////////////////////////////////////
void OccupancyMapFactory::extract1Channel(SDL_Surface* image)
{
    const unsigned char maxPixelLevel = numeric_limits<unsigned char>::max();

    unsigned char* imagePixels = (unsigned char*)(image->pixels);
    int pitch = image->pitch;

    for (unsigned int row = 0; row < metadata_.heightCells; row++)
    {
        for (unsigned int col = 0; col < metadata_.widthCells; col++)
        {
            // Find the pixel color based on the row, column, and and pitch
            unsigned char grayLevel = static_cast<unsigned char>(*(imagePixels + row * pitch + col));

            // Convert the 8 bit cost into an integer cost and store it
            Grid2d::BaseType cost = map_->grayLevel2Cost(grayLevel);
            map_->setCost(col, metadata_.heightCells - row - 1, cost);
        }
    }
}

////////////////////////////////////////////////////////////////////////////////////////////////////
void OccupancyMapFactory::extract3Channel(SDL_Surface* image)
{
    unsigned char* imagePixels = (unsigned char*)(image->pixels);
    int pitch = image->pitch;

    for (unsigned int row = 0; row < metadata_.heightCells; row++)
    {
        for (unsigned int col = 0; col < metadata_.widthCells; col++)
        {
            // Find the pixel based on the row, column, and
            // and pitch and number of channels (3)
            unsigned char* pixel = imagePixels + row * pitch + col * 3;

            // Extract the colors
            unsigned char red = *pixel;
            unsigned char green = *(pixel + 1);
            unsigned char blue = *(pixel + 2);
            unsigned char alpha = *(pixel + 3);

            MapNote* note = green > 0 ? MapNote::instanceOf(green) : nullptr;

            // Static obstacles have priority on every other note
            unsigned int cost = static_cast<unsigned int>(blue);
            cost = red > 0 ? numeric_limits<unsigned int>::max() : cost;

            map_->setCost(col, metadata_.heightCells - row - 1, cost);
        }
    }
}

} // namespace srs
