#include <srslib_framework/localization/map/occupancy/OccupancyMapFactory.hpp>

#include <limits>

#include <yaml-cpp/yaml.h>

#include <srslib_framework/exception/io/FailedToOpenFileException.hpp>
#include <srslib_framework/localization/map/occupancy/InvalidChannelNumberException.hpp>
#include <srslib_framework/localization/map/MapNote.hpp>

namespace srs {

////////////////////////////////////////////////////////////////////////////////////////////////////
// Public methods

////////////////////////////////////////////////////////////////////////////////////////////////////
OccupancyMap* OccupancyMapFactory::fromMetadata(OccupancyMetadata metadata)
{
    SDL_Surface* image = IMG_Load(metadata.occupancyFilename.c_str());
    if (!image)
    {
        throw FailedToOpenFileException(metadata.occupancyFilename);
    }

    // Copy the image data into the map structure
    metadata.widthCells = image->w;
    metadata.heightCells = image->h;

    OccupancyMap* map = new OccupancyMap(metadata.widthCells, metadata.heightCells, metadata.resolution);

    map->setLoadTime(metadata.loadTime);
    map->setNegate(metadata.negate);
    map->setOccupancyFilename(metadata.occupancyFilename);
    map->setOrigin(metadata.origin);
    map->setThresholds(metadata.thresholdFree, metadata.thresholdOccupied);

    int channels = image->format->BytesPerPixel;
    switch (channels)
    {
        case 1:
            OccupancyMapFactory::extract1Channel(image, map);
            break;

        case 3:
            OccupancyMapFactory::extract3Channel(image, map);
            break;

        default:
            throw InvalidChannelNumberException(metadata.occupancyFilename, channels);
    }

    SDL_FreeSurface(image);

    return map;
}

////////////////////////////////////////////////////////////////////////////////////////////////////
OccupancyMap* OccupancyMapFactory::fromRosCostMap2D(costmap_2d::Costmap2DROS* rosCostMap,
    double freeThreshold, double occupiedThreshold)
{
    if (!rosCostMap)
    {
        return nullptr;
    }

    costmap_2d::Costmap2D* costMap = rosCostMap->getCostmap();

    unsigned int rows = costMap->getSizeInCellsY();
    unsigned int columns = costMap->getSizeInCellsX();

    OccupancyMap* map = new OccupancyMap(columns, rows, costMap->getResolution());

    for (int row = 0; row < rows; row++)
    {
        for (int col = 0; col < columns; col++)
        {
            map->setCost(col, row, static_cast<unsigned int>(costMap->getCost(col, row)));
        }
    }

    return map;
}

////////////////////////////////////////////////////////////////////////////////////////////////////
// Private methods

////////////////////////////////////////////////////////////////////////////////////////////////////
void OccupancyMapFactory::extract1Channel(SDL_Surface* image, OccupancyMap* map)
{
    const unsigned char maxPixelLevel = numeric_limits<unsigned char>::max();

    unsigned char* imagePixels = (unsigned char*)(image->pixels);
    int pitch = image->pitch;

    OccupancyMetadata metadata = map->getMetadata();
    for (unsigned int row = 0; row < metadata.heightCells; row++)
    {
        for (unsigned int col = 0; col < metadata.widthCells; col++)
        {
            // Find the pixel color based on the row, column, and
            // and pitch
            unsigned char gray = *(imagePixels + row * pitch + col);

            unsigned int cost = static_cast<unsigned int>(
                metadata.negate ? gray : maxPixelLevel - gray);

            double percentage = cost / 255.0;

            // If the percentage is under the specified threshold
            // no additional cost is specified
            cost = percentage < metadata.thresholdFree ?
                0 : cost;

            // Static obstacles have priority on every other note
            cost = percentage > metadata.thresholdOccupied ?
                numeric_limits<unsigned int>::max() : cost;

            // In order to reduce space, the cost is stored only if
            // different from zero
            if (cost > 0)
            {
                map->setCost(col, metadata.heightCells - row - 1, cost);
            }
        }
    }
}

////////////////////////////////////////////////////////////////////////////////////////////////////
void OccupancyMapFactory::extract3Channel(SDL_Surface* image, OccupancyMap* map)
{
    unsigned char* imagePixels = (unsigned char*)(image->pixels);
    int pitch = image->pitch;

    OccupancyMetadata metadata = map->getMetadata();
    for (unsigned int row = 0; row < metadata.heightCells; row++)
    {
        for (unsigned int col = 0; col < metadata.widthCells; col++)
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

            // In order to reduce space, the cost is stored only if
            // different from zero
            if (cost > 0)
            {
                map->setCost(col, metadata.heightCells - row - 1, cost);
            }
        }
    }
}

} // namespace srs
