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

    Grid2d(unsigned int width, unsigned int height) :
        width_(width),
        height_(height)
    {}

    ~Grid2d()
    {
        clear();
    }

    void addNote(const Grid2dLocation location, void* notes = nullptr)
    {
        auto found = grid_.find(location);

        if (found != grid_.end())
        {
            Grid2dNode* node = found->second;
            node->notes = notes;
        }
        else
        {
            grid_[location] = new Grid2dNode(location, 0, notes);
        }
    }

    void addValue(const Grid2dLocation location, unsigned int cost,
        void* notes = nullptr)
    {
        auto found = grid_.find(location);

        if (found != grid_.end())
        {
            Grid2dNode* node = found->second;
            node->setCost(cost);
            node->notes = notes;
        }
        else
        {
            grid_[location] = new Grid2dNode(location, cost, notes);
        }
    }

    void clear()
    {
        for (auto gridCell : grid_)
        {
            delete gridCell.second;
        }
        grid_.clear();
//        if (grid_)
//        {
//            // Find the map notes and remove them first, because Grid will not
//            // do it for us
//            for (int row = 0; row < grid_->getHeight(); row++)
//            {
//                for (int col = 0; col < grid_->getWidth(); col++)
//                {
//                    Grid2dLocation location = Grid2dLocation(col, row);
//                    MapNote* note = reinterpret_cast<MapNote*>(grid_->getNote(location));
//
//                    if (note)
//                    {
//                        delete note;
//                    }
//                }
//            }
//
//            // Now deallocate the grid
//            delete grid_;
//        }
    }

    bool exists(const Grid2dLocation location) const
    {
        if (!isWithinBounds(location))
        {
            return false;
        }

        return grid_.count(location);
    }

    unsigned int getHeight() const
    {
        return height_;
    }

    unsigned int getCost(unsigned int c, unsigned int r) const
    {
        Grid2dLocation location = Grid2dLocation(c, r);

        auto found = grid_.find(location);
        if (found != grid_.end())
        {
            return found->second->getCost();
        }

        return 0;
    }

    // TODO: remove this when optimizing A*
    unsigned int getCost(Grid2dLocation location) const
    {
        auto found = grid_.find(location);

        if (found != grid_.end())
        {
            return found->second->getCost();
        }

        return 0;
    }

    void* getNote(Grid2dLocation location) const
    {
        auto found = grid_.find(location);

        if (found != grid_.end())
        {
            return found->second->notes;
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
                result = Grid2dLocation(location.x, location.y + 1);
                break;

            case 180:
                result = Grid2dLocation(location.x - 1, location.y);
                break;

            case 270:
                result = Grid2dLocation(location.x, location.y - 1);
                break;
        }

        return isWithinBounds(result);
    }

    unsigned int getWidth() const
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
        stream << "Grid2d {" << endl;

        stream << "(" << grid.height_ << "x" << grid.width_ << ")" << endl;

        stream << right << setw(WIDTH) << ' ';
        for (int x = 0; x < grid.width_; ++x)
        {
            stream << right << setw(WIDTH - 1) << x << ' ';
        }
        stream << endl;

        for (int y = 0; y < grid.height_; ++y)
        {
            stream << right << setw(WIDTH) << y;
            for (int x = 0; x < grid.width_; ++x)
            {
                Grid2dLocation nodeLocation;
                nodeLocation.x = x;
                nodeLocation.y = y;

                if (grid.exists(nodeLocation))
                {
                    Grid2dNode* node = grid.grid_.at(nodeLocation);
                    stream << right << setw(WIDTH - 1) << node->getCost();
                    stream << (node->notes ? '*' : ' ');
                }
                else
                {
                    stream << right << setw(WIDTH) << ". ";
                }
            }
            stream << endl;
        }

        stream << "}";

        return stream;
    }

    void setCost(unsigned int c, unsigned int r, unsigned int cost)
    {
        Grid2dLocation location = Grid2dLocation(c, r);
        auto found = grid_.find(location);

        if (found != grid_.end())
        {
            Grid2dNode* node = found->second;
            node->setCost(cost);
        }
        else
        {
            grid_[location] = new Grid2dNode(location, cost, nullptr);
        }
    }

protected:
    Grid2d() :
        width_(0),
        height_(0)
    {}

private:
    unordered_map<Grid2dLocation, Grid2dNode*> grid_;

    unsigned int width_;
    unsigned int height_;
};

} // namespace srs

#endif // GRID2D_HPP_
