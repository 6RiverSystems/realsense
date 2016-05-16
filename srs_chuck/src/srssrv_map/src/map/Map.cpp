#include <srssrv_map/map/Map.hpp>

#include <SDL/SDL_image.h>
#include <limits>

#include <tf/tf.h>
#include <tf/LinearMath/Quaternion.h>

#include <srslib_framework/utils/Filesystem.hpp>
#include <srslib_framework/graph/grid2d/Grid2dLocation.hpp>
#include <srslib_framework/search/SearchPositionNote.hpp>

#include <srslib_framework/localization/Anchor.hpp>

namespace YAML {

template<>
struct convert<srs::Anchor>
{
    static bool decode(const Node& node, srs::Anchor& anchor)
    {
        anchor.id = node["id"].as<string>();
        anchor.x = node["location"][0].as<int>();
        anchor.y = node["location"][1].as<int>();
        anchor.z = node["location"][2].as<int>();
        anchor.orientation = node["orientation"].as<int>();

        return true;
    }
};

} // YAML

namespace srs {

////////////////////////////////////////////////////////////////////////////////////////////////////
// Public methods

////////////////////////////////////////////////////////////////////////////////////////////////////
Map::Map() :
    grid_(nullptr),
    height_(0),
    mapImageFilename_(""),
    resolution_(0.0),
    width_(0)
{
}

////////////////////////////////////////////////////////////////////////////////////////////////////
Map::~Map()
{
    if (grid_)
    {
        delete grid_;
    }
}

////////////////////////////////////////////////////////////////////////////////////////////////////
void Map::getOccupancyGrid(vector<int8_t>& occupancyGrid)
{
    occupancyGrid.clear();

    if (grid_)
    {
        int8_t maxValue = numeric_limits<int8_t>::max();

        for (unsigned int row = 0; row < height_; row++)
        {
            for (unsigned int col = 0; col < width_; col++)
            {
                Grid2dLocation location = Grid2dLocation(col, row);
                float cost = (static_cast<float>(grid_->getCost(location)) / maxValue) * 100.0;

                SearchPositionNote* note = reinterpret_cast<SearchPositionNote*>(
                    grid_->getNote(location));

                if (note == &SearchPositionNote::STATIC_OBSTACLE)
                {
                    cost = maxValue;
                }

                occupancyGrid.push_back(maxValue - static_cast<int8_t>(cost));
            }
        }
    }
}

////////////////////////////////////////////////////////////////////////////////////////////////////
void Map::load(string filename)
{
    mapDocumentFilename_ = filename;

    document_ = YAML::LoadFile(mapDocumentFilename_);

    if (!document_.IsNull())
    {
        loadConfiguration();
        loadAnchors();

        if (grid_)
        {
            delete grid_;
        }
        grid_ = new Grid2d(width_, height_);

        loadCosts();
    }
    else
    {
        ROS_ERROR_STREAM("Configuration file not found: " << mapDocumentFilename_);
    }
}

////////////////////////////////////////////////////////////////////////////////////////////////////
// Private methods

////////////////////////////////////////////////////////////////////////////////////////////////////
void Map::loadAnchors()
{
    vector<Anchor> anchors = document_["anchors"].as<vector<Anchor>>();

    for (auto anchor : anchors)
    {
        cout << anchor.id << endl;
    }
}

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
    width_ = image->w;
    height_ = image->h;

    int pitch = image->pitch;
    int channels = image->format->BytesPerPixel;
    if (channels < 3)
    {
        ROS_ERROR_STREAM("The image must have RGB channels. Current channels: " << channels);
        exit(-1);
    }

    unsigned char* imagePixels = (unsigned char*)(image->pixels);
    for (unsigned int row = 0; row < height_; row++)
    {
        for (unsigned int col = 0; col < width_; col++)
        {
            // Compute mean of RGB for this pixel
            unsigned char* pixel = imagePixels + row * pitch + col * channels;

            unsigned char red = *pixel;
            unsigned char green = *(pixel + 1);
            unsigned char blue = *(pixel + 2);
            unsigned char alpha = *(pixel + 3);

            Grid2dLocation location = Grid2dLocation(col, height_ - row - 1);
            SearchPositionNote* note = nullptr;

            if (green == 255)
            {
                note = const_cast<SearchPositionNote*>(&SearchPositionNote::GO_SLOW);
            }

            // Static obstacles have priority on every other note.
            // TODO: Add a multi-note mechanism
            if (red > 0)
            {
                note = const_cast<SearchPositionNote*>(&SearchPositionNote::STATIC_OBSTACLE);
            }

            grid_->addValue(location, static_cast<unsigned int>(blue), note);
        }
    }

    SDL_FreeSurface(image);
}

} // namespace srs
