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
    grid_(nullptr),
    heightC_(0),
    heightM_(0),
    mapImageFilename_(""),
    resolution_(0.0),
    widthC_(0),
    widthM_(0)
{
}

////////////////////////////////////////////////////////////////////////////////////////////////////
Map::Map(double widthC, double heightC, double resolution) :
        grid_(nullptr),
        heightC_(heightC),
        heightM_(0),
        mapImageFilename_(""),
        resolution_(resolution),
        widthC_(widthC),
        widthM_(0)
{
    widthM_ = widthC_* resolution_;
    heightM_ = heightC_ * resolution_;

    grid_ = new Grid2d(widthC_, heightC_);
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
    c = static_cast<int>(round(x / resolution_));
    r = static_cast<int>(round(y / resolution_));
}

////////////////////////////////////////////////////////////////////////////////////////////////////
void Map::getCostsGrid(vector<int8_t>& costsGrid)
{
    costsGrid.clear();

    if (grid_)
    {
        int8_t maxValue = numeric_limits<int8_t>::max();

        for (int row = 0; row < grid_->getHeight(); row++)
        {
            for (int col = 0; col < grid_->getWidth(); col++)
            {
                Grid2dLocation location = Grid2dLocation(col, row);

                float cost = (static_cast<float>(grid_->getCost(location)) / (2.0 * maxValue)) * 100;

                // This is only for visualization purposes as
                // a static obstacle information is carried by the map note
                MapNote* note = reinterpret_cast<MapNote*>(grid_->getNote(location));
                if (note && note->staticObstacle())
                {
                    cost = 100.0;
                }

                costsGrid.push_back(static_cast<int8_t>(ceil(cost)));
            }
        }
    }
}

////////////////////////////////////////////////////////////////////////////////////////////////////
void Map::getNotesGrid(vector<int8_t>& notesGrid)
{
    notesGrid.clear();

    if (grid_)
    {
        for (int row = 0; row < grid_->getHeight(); row++)
        {
            for (int col = 0; col < grid_->getWidth(); col++)
            {
                Grid2dLocation location = Grid2dLocation(col, row);

                MapNote* note = reinterpret_cast<MapNote*>(grid_->getNote(location));
                int8_t flags = note ? note->getFlags() : 0;

                notesGrid.push_back(flags);
            }
        }
    }
}

////////////////////////////////////////////////////////////////////////////////////////////////////
void Map::getWorldCoordinates(int c, int r, double& x, double& y)
{
    x = static_cast<double>(c) * resolution_;
    y = static_cast<double>(r) * resolution_;

    // The precision is down to 1mm
    x = round(x * 1000) / 1000;
    y = round(y * 1000) / 1000;
}

////////////////////////////////////////////////////////////////////////////////////////////////////
void Map::load(string filename)
{
    mapDocumentFilename_ = filename;

    document_ = YAML::LoadFile(mapDocumentFilename_);

    if (!document_.IsNull())
    {
        loadConfiguration();
        loadCosts();
    }
    else
    {
        ROS_ERROR_STREAM("Configuration file not found: " << mapDocumentFilename_);
    }
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
// Private methods
//

////////////////////////////////////////////////////////////////////////////////////////////////////
void Map::loadConfiguration()
{
    ROS_INFO_STREAM("Loading map configuration " << mapDocumentFilename_);

    try
    {
        resolution_ = document_["resolution"].as<double>();
    }
    catch (YAML::InvalidScalar& e)
    {
        ROS_ERROR("The map does not contain a resolution tag or it is invalid.");
        exit(-1);
    }

    try
    {
        mapImageFilename_ = document_["image"].as<string>();

        if (mapImageFilename_.size() == 0)
        {
            ROS_ERROR("The image tag cannot be an empty string.");
            exit(-1);
        }

        if (mapImageFilename_[0] != '/')
        {
            mapImageFilename_ = Filesystem::dirname(mapDocumentFilename_) +
                "/" + mapImageFilename_;
        }

    }
    catch (YAML::InvalidScalar& e)
    {
        ROS_ERROR("The map does not contain an image tag or it is invalid.");
        exit(-1);
    }

    origin_ = tf::createQuaternionFromYaw(0.0);
    orientation_ = tf::createQuaternionFromYaw(0.0);
}

////////////////////////////////////////////////////////////////////////////////////////////////////
void Map::loadCosts()
{
    ROS_INFO_STREAM("Loading map from image " << mapImageFilename_);

    SDL_Surface* image;

    image = IMG_Load(mapImageFilename_.c_str());
    if (!image)
    {
        ROS_ERROR_STREAM("Failed to open the image file \"" << mapImageFilename_ + "\"");
        throw;
    }

    // Copy the image data into the map structure
    widthC_ = image->w;
    heightC_ = image->h;
    widthM_ = widthC_ * resolution_;
    heightM_ = heightC_ * resolution_;

    if (grid_)
    {
        delete grid_;
    }
    grid_ = new Grid2d(widthC_, heightC_);

    int pitch = image->pitch;
    int channels = image->format->BytesPerPixel;
    if (channels < 3)
    {
        ROS_ERROR_STREAM("The image must have RGB channels. Current channels: " << channels);
        exit(-1);
    }

    int8_t maxValue = numeric_limits<int8_t>::max();

    unsigned char* imagePixels = (unsigned char*)(image->pixels);
    for (unsigned int row = 0; row < heightC_; row++)
    {
        for (unsigned int col = 0; col < widthC_; col++)
        {
            // Compute mean of RGB for this pixel
            unsigned char* pixel = imagePixels + row * pitch + col * channels;

            unsigned char red = *pixel;
            unsigned char green = *(pixel + 1);
            unsigned char blue = *(pixel + 2);
            unsigned char alpha = *(pixel + 3);

            Grid2dLocation location = Grid2dLocation(col, heightC_ - row - 1);
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
