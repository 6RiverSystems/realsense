#include <srslib_framework/localization/map/Map.hpp>

#include <SDL/SDL_image.h>
#include <limits>

#include <tf/tf.h>
#include <tf/LinearMath/Quaternion.h>

#include <srslib_framework/utils/Filesystem.hpp>
#include <srslib_framework/graph/grid2d/Grid2dLocation.hpp>

#include <srslib_framework/localization/map/Anchor.hpp>

namespace srs {

////////////////////////////////////////////////////////////////////////////////////////////////////
// Public methods

////////////////////////////////////////////////////////////////////////////////////////////////////
Map::Map() :
    grid_(nullptr)
{
}

////////////////////////////////////////////////////////////////////////////////////////////////////
Map::Map(double widthC, double heightC, double resolution) :
        grid_(nullptr)
{
    metadata_.heightCells = heightC;
    metadata_.widthCells = widthC;
    metadata_.resolution = resolution;

    metadata_.widthM = widthC * resolution;
    metadata_.heightM = heightC * resolution;

    grid_ = new Grid2d(widthC, heightC);
}

////////////////////////////////////////////////////////////////////////////////////////////////////
Map::~Map()
{
    if (grid_)
    {
        // Find the map notes and remove them first, because Grid will not
        // do it for us
        for (int row = 0; row < grid_->getHeight(); row++)
        {
            for (int col = 0; col < grid_->getWidth(); col++)
            {
                Grid2dLocation location = Grid2dLocation(col, row);
                MapNote* note = reinterpret_cast<MapNote*>(grid_->getNote(location));

                if (note)
                {
                    delete note;
                }
            }
        }

        // Now deallocate the grid
        delete grid_;
    }
}

////////////////////////////////////////////////////////////////////////////////////////////////////
void Map::getMapCoordinates(double x, double y, int& c, int& r)
{
    c = static_cast<int>(round(x / metadata_.resolution));
    r = static_cast<int>(round(y / metadata_.resolution));
}

////////////////////////////////////////////////////////////////////////////////////////////////////
void Map::getWorldCoordinates(int c, int r, double& x, double& y)
{
    x = static_cast<double>(c) * metadata_.resolution;
    y = static_cast<double>(r) * metadata_.resolution;

    // The precision is down to 1mm
    x = round(x * 1000) / 1000;
    y = round(y * 1000) / 1000;
}

////////////////////////////////////////////////////////////////////////////////////////////////////
void Map::load(string filename)
{
    metadata_.mapDocumentFilename = filename;

    document_ = YAML::LoadFile(metadata_.mapDocumentFilename);

    if (!document_.IsNull())
    {
        loadConfiguration();
        loadCosts();
    }
    else
    {
        ROS_ERROR_STREAM("Configuration file not found: " << metadata_.mapDocumentFilename);
    }
}

////////////////////////////////////////////////////////////////////////////////////////////////////
void Map::setCost(int c, int r, unsigned int cost)
{
    Grid2dLocation location = Grid2dLocation(c, r);
    grid_->addValue(location, cost);
}

////////////////////////////////////////////////////////////////////////////////////////////////////
void Map::setGrid(const vector<int8_t>& costsGrid, const vector<int8_t>& notesGrid)
{
    if (grid_)
    {
        auto costsIterator = costsGrid.begin();
        auto notesIterator = notesGrid.begin();

        for (int row = 0; row < grid_->getHeight(); row++)
        {
            for (int col = 0; col < grid_->getWidth(); col++)
            {
                Grid2dLocation location = Grid2dLocation(col, row);

                char flags = *notesIterator;
                float cost = (static_cast<float>(*costsIterator) / 100) * 255;

                MapNote* note = reinterpret_cast<MapNote*>(grid_->getNote(location));
                if (note)
                {
                    note->add(flags);
                }
                else
                {
                    note = MapNote::instanceOf(flags);
                }

                grid_->addValue(location, static_cast<unsigned int>(cost), note);

                costsIterator++;
                notesIterator++;
            }
        }
    }
}

////////////////////////////////////////////////////////////////////////////////////////////////////
void Map::setObstruction(int c, int r)
{
    Grid2dLocation location = Grid2dLocation(c, r);
    void* note = reinterpret_cast<void*>(MapNote::instanceOf(MapNote::STATIC_OBSTACLE));

    grid_->addValue(location, numeric_limits<unsigned int>::max(), note);
}

////////////////////////////////////////////////////////////////////////////////////////////////////
// Private methods
//

////////////////////////////////////////////////////////////////////////////////////////////////////
void Map::loadConfiguration()
{
    ROS_INFO_STREAM("Loading map configuration: " << metadata_.mapDocumentFilename);

    try
    {
        metadata_.resolution = document_["resolution"].as<double>();
    }
    catch (YAML::InvalidScalar& e)
    {
        ROS_ERROR("The map does not contain a resolution tag or it is invalid.");
        exit(-1);
    }

    try
    {
        metadata_.mapImageFilename = document_["image"].as<string>();

        if (metadata_.mapImageFilename.size() == 0)
        {
            ROS_ERROR("The image tag cannot be an empty string.");
            exit(-1);
        }

        if (metadata_.mapImageFilename[0] != '/')
        {
            metadata_.mapImageFilename = Filesystem::dirname(metadata_.mapDocumentFilename) +
                "/" + metadata_.mapImageFilename;
        }

    }
    catch (YAML::InvalidScalar& e)
    {
        ROS_ERROR("The map does not contain an image tag or it is invalid.");
        exit(-1);
    }
}

////////////////////////////////////////////////////////////////////////////////////////////////////
void Map::loadCosts()
{
    ROS_INFO_STREAM("Loading map from image " << metadata_.mapImageFilename);

    SDL_Surface* image;

    image = IMG_Load(metadata_.mapImageFilename.c_str());
    if (!image)
    {
        ROS_ERROR_STREAM("Failed to open the image file \"" << metadata_.mapImageFilename + "\"");
        throw;
    }

    // Copy the image data into the map structure
    metadata_.widthCells = image->w;
    metadata_.heightCells = image->h;
    metadata_.widthM = image->w * metadata_.resolution;
    metadata_.heightM = image->h * metadata_.resolution;

    if (grid_)
    {
        delete grid_;
    }
    grid_ = new Grid2d(metadata_.widthCells, metadata_.heightCells);

    int pitch = image->pitch;
    int channels = image->format->BytesPerPixel;
    if (channels < 3)
    {
        ROS_ERROR_STREAM("The image must have RGB channels. Current channels: " << channels);
        exit(-1);
    }

    int8_t maxValue = numeric_limits<int8_t>::max();

    unsigned char* imagePixels = (unsigned char*)(image->pixels);
    for (unsigned int row = 0; row < metadata_.heightCells; row++)
    {
        for (unsigned int col = 0; col < metadata_.widthCells; col++)
        {
            // Compute mean of RGB for this pixel
            unsigned char* pixel = imagePixels + row * pitch + col * channels;

            unsigned char red = *pixel;
            unsigned char green = *(pixel + 1);
            unsigned char blue = *(pixel + 2);
            unsigned char alpha = *(pixel + 3);

            Grid2dLocation location = Grid2dLocation(col, metadata_.heightCells - row - 1);
            MapNote* note = MapNote::instanceOf(green);

            // Static obstacles have priority on every other note.
            if (red > 0)
            {
                note->join(MapNote::STATIC_OBSTACLE);
            }

            unsigned int cost = static_cast<unsigned int>(blue);
            grid_->addValue(location, cost, note);
        }
    }

    SDL_FreeSurface(image);
}

////////////////////////////////////////////////////////////////////////////////////////////////////
// Global operators

////////////////////////////////////////////////////////////////////////////////////////////////////
ostream& operator<<(ostream& stream, const Map& map)
{
    int8_t maxValue = numeric_limits<int8_t>::max();

    stream << "Map (" << map.grid_->getHeight() << "x" << map.grid_->getWidth() << ")" << endl;

    for (int row = map.grid_->getHeight() - 1; row >= 0; row--)
    {
        for (int col = 0; col < map.grid_->getWidth(); col++)
        {
            Grid2dLocation location = Grid2dLocation(col, row);

            MapNote* note = reinterpret_cast<MapNote*>(map.grid_->getNote(location));
            float floatCost = (static_cast<float>(map.grid_->getCost(location)) / maxValue) * 100.0;

            char cost = floatCost > 0.0 ? '+' : '.';

            if (note && note->staticObstacle())
            {
                cost = '#';
            }

            stream << cost;
        }

        stream << endl;
    }

    return stream;
}

} // namespace srs
