#include <srslib_framework/datastructure/graph/grid2d/Grid2d.hpp>

namespace srs {

////////////////////////////////////////////////////////////////////////////////////////////////////
// Public methods

////////////////////////////////////////////////////////////////////////////////////////////////////
void Grid2d::addCost(const Location& location, unsigned int cost)
{
    auto found = grid_.find(location);

    Node* node;
    if (found != grid_.end())
    {
        node = found->second;
        node->cost = BasicMath::noOverflowAdd(node->cost, cost);
    }
    else
    {
        node = new Node(location, cost, 0);
        grid_[location] = node;
    }

    if (aggregate_)
    {
        updateAllAggregate(node);
    }
}

////////////////////////////////////////////////////////////////////////////////////////////////////
void Grid2d::clear()
{
    for (auto cell : grid_)
    {
        delete cell.second;
    }

    grid_.clear();
}

////////////////////////////////////////////////////////////////////////////////////////////////////
unsigned int Grid2d::getCost(const Location& location) const
{
    auto found = grid_.find(location);
    if (found != grid_.end())
    {
        return found->second->cost;
    }

    return 0;
}

////////////////////////////////////////////////////////////////////////////////////////////////////
bool Grid2d::getNeighbor(const Location& location, int orientation, Location& result) const
{
    switch (orientation)
    {
        case 0:
            result = Location(location.x + 1, location.y);
            break;

        case 90:
            result = Location(location.x, location.y + 1);
            break;

        case 180:
            result = Location(location.x - 1, location.y);
            break;

        case 270:
            result = Location(location.x, location.y - 1);
            break;
    }

    return isWithinBounds(result);
}

////////////////////////////////////////////////////////////////////////////////////////////////////
ostream& operator<<(ostream& stream, const Grid2d& grid)
{
    constexpr int WIDTH = 6;
    stream << "Grid2d {" << endl;

    stream << "(" << grid.height_ << "x" << grid.width_ << ")" << endl;

    stream << right << setw(WIDTH) << ' ';
    for (unsigned int x = 0; x < grid.width_; ++x)
    {
        stream << right << setw(WIDTH) << x;
    }
    stream << endl;

    for (unsigned int y = 0; y < grid.height_; ++y)
    {
        stream << right << setw(WIDTH) << y;
        for (unsigned int x = 0; x < grid.width_; ++x)
        {
            Grid2d::Location nodeLocation(x, y);
            if (grid.exists(nodeLocation))
            {
                Grid2d::Node* node = grid.grid_.at(nodeLocation);
                stream << right << setw(WIDTH) << node->cost;
            }
            else
            {
                stream << right << setw(WIDTH) << ".";
            }
        }
        stream << endl;
    }

    stream << endl;

    stream << right << setw(WIDTH) << ' ';
    for (unsigned int x = 0; x < grid.width_; ++x)
    {
        stream << right << setw(WIDTH) << x;
    }
    stream << endl;

    for (unsigned int y = 0; y < grid.height_; ++y)
    {
        stream << right << setw(WIDTH) << y;
        for (unsigned int x = 0; x < grid.width_; ++x)
        {
            Grid2d::Location nodeLocation(x, y);
            if (grid.exists(nodeLocation))
            {
                Grid2d::Node* node = grid.grid_.at(nodeLocation);
                stream << right << setw(WIDTH) << node->aggregateCost;
            }
            else
            {
                stream << right << setw(WIDTH) << ".";
            }
        }
        stream << endl;
    }

    stream << "}";

    return stream;
}

////////////////////////////////////////////////////////////////////////////////////////////////////
void Grid2d::setAggregateSize(unsigned int width, unsigned int height)
{
    aggregate_ = true;
    aggregateWidth_ = width;
    aggregateHeight_ = height;

    updateAllAggregate(nullptr);
}

////////////////////////////////////////////////////////////////////////////////////////////////////
void Grid2d::setCost(const Location& location, unsigned int cost)
{
    auto found = grid_.find(location);

    Node* node;
    if (found != grid_.end())
    {
        node = found->second;
        node->cost = cost;
    }
    else
    {
        node = new Node(location, cost, 0);
        grid_[location] = node;
    }

    if (aggregate_)
    {
        updateAllAggregate(node);
    }
}

////////////////////////////////////////////////////////////////////////////////////////////////////
// Private methods

////////////////////////////////////////////////////////////////////////////////////////////////////
void Grid2d::calculateAggregateArea(unsigned int x0, unsigned int y0,
    unsigned int& xi, unsigned int& xf,
    unsigned int& yi, unsigned int& yf)
{
    xi = x0 - round(static_cast<double>(aggregateWidth_) / 2);
    xi = xi < 0 ? 0 : xi;

    xf = xi + aggregateWidth_ * 2;
    xf = xf >= width_ ? width_ - 1 : xf;

    yi = y0 - round(static_cast<double>(aggregateHeight_) / 2);
    yi = yi < 0 ? 0 : yi;

    yf = yi + aggregateHeight_ * 2;
    yf = yf >= height_ ? height_ - 1 : yf;
}

////////////////////////////////////////////////////////////////////////////////////////////////////
void Grid2d::updateAllAggregate(Node* node)
{
    unsigned int xi;
    unsigned int xf;
    unsigned int yi;
    unsigned int yf;

    if (node)
    {
        calculateAggregateArea(node->location.x, node->location.y, xi, xf, yi, yf);
    }
    else
    {
        xi = 0;
        xf = width_ - 1;

        yi = 0;
        yf = height_ - 1;
    }

    unsigned int aggregateCost = 0;
    for (unsigned int y = yi; y <= yf; ++y)
    {
        for (unsigned int x = xi; x <= xf; ++x)
        {
            auto found = grid_.find(Location(x, y));
            if (found != grid_.end())
            {
                updateNodeAggregate(found->second);
            }
        }
    }
}

////////////////////////////////////////////////////////////////////////////////////////////////////
void Grid2d::updateNodeAggregate(Node* node)
{
    unsigned int xi;
    unsigned int xf;
    unsigned int yi;
    unsigned int yf;

    calculateAggregateArea(node->location.x, node->location.y, xi, xf, yi, yf);

    unsigned int aggregateCost = 0;
    for (unsigned int y = yi; y <= yf; ++y)
    {
        for (unsigned int x = xi; x <= xf; ++x)
        {
            auto found = grid_.find(Location(x, y));
            if (found != grid_.end())
            {
                aggregateCost = BasicMath::noOverflowAdd(aggregateCost, found->second->cost);
            }
        }
    }

    node->aggregateCost = aggregateCost;
}

} // namespace srs
