/*
 * (c) Copyright 2015-2016 River Systems, all rights reserved.
 *
 * This is proprietary software, unauthorized distribution is not permitted.
 */
#ifndef GRID2D_HPP_
#define GRID2D_HPP_

#include <sstream>
#include <iostream>
#include <iomanip>
#include <vector>
#include <unordered_map>
using namespace std;

#include <srslib_framework/graph/grid2d/Grid2dLocation.hpp>
#include <srslib_framework/graph/grid2d/Grid2dNode.hpp>

namespace srs {

class Grid2d
{
public:
    typedef Grid2dLocation LocationType;

    Grid2d(int size) :
        width_(size),
        height_(size)
    {}

    Grid2d(int width, int height) :
        width_(width),
        height_(height)
    {}

    void addValue(const Grid2dLocation location, const unsigned int cost, void* notes = nullptr)
    {
        Grid2dNode* newNode = new Grid2dNode(location, cost, notes);
        grid_[location] = newNode;
    }

    void clear()
    {
        for (auto gridCell : grid_)
        {
            delete gridCell.second;
        }
        grid_.clear();
    }

    bool exists(const Grid2dLocation location) const
    {
        if (!isWithinBounds(location))
        {
            return false;
        }

        return grid_.count(location);
    }

    int getHeight() const
    {
        return height_;
    }

    unsigned int getCost(Grid2dLocation location) const
    {
        if (!isWithinBounds(location))
        {
            throw;
        }

        auto node = grid_.find(location);
        if (node != grid_.end())
        {
            return node->second->cost;
        }
        return 0;
    }

    void* getNote(Grid2dLocation location) const
    {
        if (!isWithinBounds(location))
        {
            throw;
        }

        auto node = grid_.find(location);
        if (node != grid_.end())
        {
            return node->second->notes;
        }
        return nullptr;
    }

    bool getNeighbor(Grid2dLocation location, int orientation, Grid2dLocation& result) const
    {
        switch (orientation)
        {
            case 0:
                result = Grid2dLocation(location.x + 1, location.y);
                break;

            case 90:
                result = Grid2dLocation(location.x, location.y - 1);
                break;

            case 180:
                result = Grid2dLocation(location.x - 1, location.y);
                break;

            case 270:
                result = Grid2dLocation(location.x, location.y + 1);
                break;
        }

        return isWithinBounds(result);
    }

    int getWidth() const
    {
        return width_;
    }

    bool isWithinBounds(const Grid2dLocation location) const
    {
        return (0 <= location.x && location.x < width_) &&
            (0 <= location.y && location.y < height_);
    }

    friend ostream& operator<<(ostream& stream, const Grid2d& grid)
    {
        constexpr int WIDTH = 6;
        stream << "Grid2d {" << '\n';

        cout << "(" << grid.height_ << "x" << grid.width_ << ")" << endl;

        cout << right << setw(WIDTH) << ' ';
        for (int x = 0; x < grid.width_; ++x)
        {
            cout << right << setw(WIDTH - 1) << x << ' ';
        }
        cout << endl;

        for (int y = 0; y < grid.height_; ++y)
        {
            cout << right << setw(WIDTH) << y;
            for (int x = 0; x < grid.width_; ++x)
            {
                Grid2dLocation nodeLocation;
                nodeLocation.x = x;
                nodeLocation.y = y;

                if (grid.exists(nodeLocation))
                {
                    Grid2dNode* node = grid.grid_.at(nodeLocation);
                    cout << right << setw(WIDTH - 1) << node->cost;
                    cout << (node->notes ? '*' : ' ');
                }
                else
                {
                    cout << right << setw(WIDTH) << ". ";
                }
            }
            cout << endl;
        }

        stream << "}";

        return stream;
    }

protected:
    Grid2d() :
        width_(0),
        height_(0)
    {}

private:
    unordered_map<Grid2dLocation, Grid2dNode*> grid_;

    int width_;
    int height_;
};

} // namespace srs

#endif // GRID2D_HPP_
