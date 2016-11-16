#include <srslib_framework/localization/map/occupancy/OccupancyMapFactory.hpp>

#include <limits>

#include <yaml-cpp/yaml.h>

#include <srslib_framework/exception/io/FailedToOpenFileException.hpp>
#include <srslib_framework/localization/map/occupancy/exception/InvalidChannelNumberException.hpp>

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

    map_ = new OccupancyMap(columns, rows, costMap->getResolution(),
        Pose<>(costMap->getOriginX(), costMap->getOriginY(), 0));
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
OccupancyMap* OccupancyMapFactory::fromGrid2d(Grid2d* grid, double resolution, Pose<> origin)
{
    map_ = new OccupancyMap(grid, resolution, origin);
    metadata_ = map_->getMetadata();

    return map_;
}

////////////////////////////////////////////////////////////////////////////////////////////////////
OccupancyMap* OccupancyMapFactory::fromMetadata(const OccupancyMetadata& metadata)
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
            extractMonoChannel(image);
            break;

        case 3:
            extractRGBChannel(image);
            break;

        case 4:
            extractRGBAChannel(image);
            break;

        default:
            throw InvalidChannelNumberException(metadata_.occupancyFilename, channels);
    }

    SDL_FreeSurface(image);

    return map_;
}

////////////////////////////////////////////////////////////////////////////////////////////////////
OccupancyMap* OccupancyMapFactory::fromMetadata(const OccupancyMetadata& metadata,
    const vector<int8_t>& occupancy)
{
    OccupancyMap* map = new OccupancyMap(metadata);

    auto occupancyIterator = occupancy.begin();

    for (int row = 0; row < metadata.heightCells; row++)
    {
        for (int col = 0; col < metadata.widthCells; col++)
        {
            // Convert the 8 bit cost into an integer cost and store it
            int8_t grayLevel = *occupancyIterator;
            Grid2d::BaseType cost = map->gray2Cost(static_cast<char>(grayLevel));
            map->setCost(col, row, cost);

            occupancyIterator++;
        }
    }

    return map;
}

////////////////////////////////////////////////////////////////////////////////////////////////////
// Private methods

////////////////////////////////////////////////////////////////////////////////////////////////////
void OccupancyMapFactory::extractMonoChannel(SDL_Surface* image)
{
    const unsigned char maxPixelLevel = numeric_limits<unsigned char>::max();

    unsigned char* imagePixels = (unsigned char*)(image->pixels);
    int pitch = image->pitch;

    for (unsigned int row = 0; row < metadata_.heightCells; row++)
    {
        for (unsigned int col = 0; col < metadata_.widthCells; col++)
        {
            // Find the pixel color based on the row, column, and and pitch
            // Pgm pixels range from 0 to 255.
            unsigned char grayLevel = static_cast<unsigned char>(*(imagePixels + row * pitch + col));

            // Convert the 8 bit cost into a cost and store it
            Grid2d::BaseType cost = map_->gray2Cost(grayLevel);

            map_->setCost(col, metadata_.heightCells - row - 1, cost);
        }
    }
}

////////////////////////////////////////////////////////////////////////////////////////////////////
void OccupancyMapFactory::extractRGBChannel(SDL_Surface* image)
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

            // Convert the RGB pixel color into a cost and store it
            Grid2d::BaseType cost = map_->rgb2Cost(red, green, blue);
            map_->setCost(col, metadata_.heightCells - row - 1, cost);
        }
    }
}

////////////////////////////////////////////////////////////////////////////////////////////////////
void OccupancyMapFactory::extractRGBAChannel(SDL_Surface* image)
{
    unsigned char* imagePixels = (unsigned char*)(image->pixels);
    int pitch = image->pitch;

    for (unsigned int row = 0; row < metadata_.heightCells; row++)
    {
        for (unsigned int col = 0; col < metadata_.widthCells; col++)
        {
            // Find the pixel based on the row, column, and
            // and pitch and number of channels (3)
            unsigned char* pixel = imagePixels + row * pitch + col * 4;

            // Extract the colors
            unsigned char red = *pixel;
            unsigned char green = *(pixel + 1);
            unsigned char blue = *(pixel + 2);
            unsigned char alpha = *(pixel + 3);

            // Convert the RGBA pixel color into a cost and store it
            Grid2d::BaseType cost = map_->rgba2Cost(red, green, blue, alpha);
            map_->setCost(col, metadata_.heightCells - row - 1, cost);
        }
    }
}

} // namespace srs
