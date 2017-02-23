#include <srslib_framework/localization/map/occupancy/OccupancyMapFactory.hpp>

#include <limits>

#include <yaml-cpp/yaml.h>

#include <srslib_framework/exception/io/FailedToOpenFileException.hpp>
#include <srslib_framework/localization/map/occupancy/exception/InvalidChannelNumberException.hpp>

namespace srs {

////////////////////////////////////////////////////////////////////////////////////////////////////
// Public methods

////////////////////////////////////////////////////////////////////////////////////////////////////
OccupancyMap* OccupancyMapFactory::fromGrid2d(SimpleGrid2d* occupancy,
    Pose<> origin, double resolution)
{
    metadata_ = OccupancyMetadata(occupancy->getWidth(), occupancy->getHeight(),
        origin, resolution,
        "",
        0.9, 0.1, false);

    map_ = new OccupancyMap(metadata_, occupancy);

    return map_;
}

////////////////////////////////////////////////////////////////////////////////////////////////////
OccupancyMap* OccupancyMapFactory::fromMetadata(const OccupancyMetadata& metadata)
{
    map_ = nullptr;
    metadata_ = metadata;

    SDL_Surface* image = nullptr;
    try
    {
        image = IMG_Load(metadata.occupancyFilename.c_str());
    }
    catch (exception& e)
    {
        throw FailedToOpenFileException(metadata_.occupancyFilename);
    }

    if (!image)
    {
        throw FailedToOpenFileException(metadata_.occupancyFilename);
    }

    // Copy the image data into the map structure
    metadata_.widthCells = image->w;
    metadata_.heightCells = image->h;

    metadata_.widthM = MeasurementMath::cells2M(metadata_.widthCells, metadata_.resolution);
    metadata_.heightM = MeasurementMath::cells2M(metadata_.heightCells, metadata_.resolution);

    SimpleGrid2d* grid = new SimpleGrid2d(metadata_.widthCells, metadata_.heightCells);
    map_ = new OccupancyMap(metadata_, grid);

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
    map_ = nullptr;
    metadata_ = metadata;

    SimpleGrid2d* grid = new SimpleGrid2d(metadata_.widthCells, metadata_.heightCells);
    map_ = new OccupancyMap(metadata_, grid);

    auto occupancyIterator = occupancy.begin();

    for (int row = 0; row < metadata_.heightCells; row++)
    {
        for (int col = 0; col < metadata_.widthCells; col++)
        {
            // Convert the 8 bit cost into an integer cost and store it
            int8_t grayLevel = *occupancyIterator;
            map_->costSet(col, row, static_cast<SimpleGrid2d::BaseType>(grayLevel));

            occupancyIterator++;
        }
    }

    return map_;
}

////////////////////////////////////////////////////////////////////////////////////////////////////
OccupancyMap* OccupancyMapFactory::fromMetadataRawCost(const OccupancyMetadata& metadata)
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

    SimpleGrid2d* grid = new SimpleGrid2d(metadata_.widthCells, metadata_.heightCells);
    map_ = new OccupancyMap(metadata_, grid);

    int channels = image->format->BytesPerPixel;
    if (channels == 1)
    {
        extractMonoChannelRaw(image);
    }
    else
    {
        throw InvalidChannelNumberException(metadata_.occupancyFilename, channels);
    }

    SDL_FreeSurface(image);

    return map_;
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
            // PGM pixels range from 0 to 255
            unsigned char grayLevel = static_cast<unsigned char>(*(imagePixels + row * pitch + col));

            // Convert the 8 bit cost into a cost and store it
            SimpleGrid2d::BaseType cost = map_->gray2Cost(grayLevel);
            map_->costSet(col, metadata_.heightCells - row - 1, cost);
        }
    }
}

////////////////////////////////////////////////////////////////////////////////////////////////////
void OccupancyMapFactory::extractMonoChannelRaw(SDL_Surface* image)
{
    const unsigned char maxPixelLevel = numeric_limits<unsigned char>::max();

    unsigned char* imagePixels = (unsigned char*)(image->pixels);
    int pitch = image->pitch;

    for (unsigned int row = 0; row < metadata_.heightCells; row++)
    {
        for (unsigned int col = 0; col < metadata_.widthCells; col++)
        {
            // Find the pixel color based on the row, column, and and pitch
            // PGM pixels range from 0 to 255
            SimpleGrid2d::BaseType cost = static_cast<unsigned char>(*(imagePixels + row * pitch + col));
            map_->costSet(col, metadata_.heightCells - row - 1, cost);
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
            SimpleGrid2d::BaseType cost = map_->rgb2Cost(red, green, blue);
            map_->costSet(col, metadata_.heightCells - row - 1, cost);
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
            SimpleGrid2d::BaseType cost = map_->rgba2Cost(red, green, blue, alpha);
            map_->costSet(col, metadata_.heightCells - row - 1, cost);
        }
    }
}

} // namespace srs
