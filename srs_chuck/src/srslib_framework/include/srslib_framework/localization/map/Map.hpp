/*
 * (c) Copyright 2015-2016 River Systems, all rights reserved.
 *
 * This is proprietary software, unauthorized distribution is not permitted.
 */
#ifndef MAP_HPP_
#define MAP_HPP_

#include <string>
#include <sstream>
using namespace std;

#include <SDL/SDL_image.h>
#include <yaml-cpp/yaml.h>
#include <tf/tf.h>

#include <srslib_framework/graph/grid2d/Grid2d.hpp>
#include <srslib_framework/localization/map/MapMetadata.hpp>
#include <srslib_framework/localization/map/MapNote.hpp>

namespace srs {

class Map
{
public:
    Map();
    Map(double widthMeters, double heightMeters, double resolution);

    ~Map();

    int getHeightCells()
    {
        return metadata_.heightCells;
    }

    double getHeightMeters()
    {
        return metadata_.heightM;
    }

    Grid2d* getGrid()
    {
        return grid_;
    }

    void getMapCoordinates(double x, double y, int& c, int& r);

    MapMetadata getMetadata() const
    {
        return metadata_;
    }

    float getResolution()
    {
        return metadata_.resolution;
    }

    int getWidthCells()
    {
        return metadata_.widthCells;
    }

    double getWidthMeters()
    {
        return metadata_.widthM;
    }

    void getWorldCoordinates(int c, int r, double& x, double& y);

    void load(string filename, double loadTime = 0);

    friend ostream& operator<<(ostream& stream, const Map& map);

    void setCost(int c, int r, unsigned int cost);
    void setGrid(const vector<int8_t>& costsGrid, const vector<int8_t>& notesGrid);
    void setObstruction(int c, int r);

private:
    void extract1Channel(SDL_Surface* image);
    void extract3Channel(SDL_Surface* image);

    void loadConfiguration();
    void loadCosts();

    YAML::Node document_;

    MapMetadata metadata_;

    Grid2d* grid_;
};

ostream& operator<<(ostream& stream, const Map& map);

} // namespace srs

#endif // MAP_HPP_
