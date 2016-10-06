#include <srslib_framework/datastructure/graph/grid2d/Grid2d.hpp>

namespace srs {

////////////////////////////////////////////////////////////////////////////////////////////////////
// Public methods

const int Grid2d::MIN_COST = 0;
const int Grid2d::MAX_COST = numeric_limits<int>::max();

////////////////////////////////////////////////////////////////////////////////////////////////////
void Grid2d::addCost(const Location& location, int cost)
{
    auto found = grid_.find(location);

    Node* node;
    int oldCost = MIN_COST;

    if (found != grid_.end())
    {
        node = found->second;
        oldCost = node->cost;
        node->cost = BasicMath::noOverflowAdd<int>(node->cost, cost);
    }
    else
    {
        node = new Node(location, cost, MIN_COST);
        grid_[location] = node;

        if (aggregate_)
        {
            node->aggregateCost = BasicMath::noOverflowAdd(calculateAggregateCost(node), -cost);
        }
    }

    if (aggregate_)
    {
        updateNodeAggregate(node, oldCost);
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
int Grid2d::getAggregateCost(const Location& location) const
{
    auto found = grid_.find(location);
    if (found != grid_.end())
    {
        return found->second->aggregateCost;
    }

    return MIN_COST;
}

////////////////////////////////////////////////////////////////////////////////////////////////////
int Grid2d::getCost(const Location& location) const
{
    auto found = grid_.find(location);
    if (found != grid_.end())
    {
        return found->second->cost;
    }

    return MIN_COST;
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
    for (int x = 0; x < grid.width_; ++x)
    {
        stream << right << setw(WIDTH) << x;
    }
    stream << endl;

    for (int y = 0; y < grid.height_; ++y)
    {
        stream << right << setw(WIDTH) << y;
        for (int x = 0; x < grid.width_; ++x)
        {
            Grid2d::Location nodeLocation(x, y);
            if (grid.exists(nodeLocation))
            {
                Grid2d::Node* node = grid.grid_.at(nodeLocation);

                if (node->cost == Grid2d::MAX_COST)
                {
                    stream << right << setw(WIDTH) << "#";
                }
                else if (node->cost == Grid2d::MIN_COST)
                {
                    stream << right << setw(WIDTH) << ".";
                }
                else
                {
                    stream << right << setw(WIDTH) << node->cost;
                }
            }
            else
            {
                stream << right << setw(WIDTH) << ".";
            }
        }
        stream << endl;
    }

    if (grid.aggregate_)
    {
        stream << endl;

        stream << right << setw(WIDTH) << ' ';
        for (int x = 0; x < grid.width_; ++x)
        {
            stream << right << setw(WIDTH) << x;
        }
        stream << endl;

        for (int y = 0; y < grid.height_; ++y)
        {
            stream << right << setw(WIDTH) << y;
            for (int x = 0; x < grid.width_; ++x)
            {
                Grid2d::Location nodeLocation(x, y);
                if (grid.exists(nodeLocation))
                {
                    Grid2d::Node* node = grid.grid_.at(nodeLocation);

                    if (node->aggregateCost == Grid2d::MAX_COST)
                    {
                        stream << right << setw(WIDTH) << "#";
                    }
                    else if (node->aggregateCost == Grid2d::MIN_COST)
                    {
                        stream << right << setw(WIDTH) << ".";
                    }
                    else
                    {
                        stream << right << setw(WIDTH) << node->aggregateCost;
                    }
                }
                else
                {
                    stream << right << setw(WIDTH) << ".";
                }
            }
            stream << endl;
        }
    }

    stream << "}";

    return stream;
}

////////////////////////////////////////////////////////////////////////////////////////////////////
void Grid2d::setAggregateSize(unsigned int width, unsigned int height)
{
    if (width > 0 && height > 0)
    {
        aggregate_ = true;
        aggregateWidth_ = width;
        aggregateHeight_ = height;

        updateAllAggregate();
    }
    else
    {
        aggregate_ = false;
    }
}

////////////////////////////////////////////////////////////////////////////////////////////////////
void Grid2d::setCost(const Location& location, int newCost)
{
    auto found = grid_.find(location);

    Node* node;
    int oldCost = MIN_COST;

    if (found != grid_.end())
    {
        node = found->second;
        oldCost = node->cost;
        node->cost = newCost;
    }
    else
    {
        node = new Node(location, newCost, MIN_COST);
        grid_[location] = node;

        if (aggregate_)
        {
            node->aggregateCost = BasicMath::noOverflowAdd(calculateAggregateCost(node), -newCost);
        }
    }

    if (aggregate_)
    {
        updateNodeAggregate(node, oldCost);
    }
}

////////////////////////////////////////////////////////////////////////////////////////////////////
// Private methods

////////////////////////////////////////////////////////////////////////////////////////////////////
int Grid2d::calculateAggregateCost(Node* node)
{
    int xi;
    int xf;
    int yi;
    int yf;
    calculateAggregateArea(node->location.x, node->location.y, xi, xf, yi, yf);

    int aggregateCost = MIN_COST;
    for (int y = yi; y <= yf; ++y)
    {
        for (int x = xi; x <= xf; ++x)
        {
            auto found = grid_.find(Location(x, y));
            if (found != grid_.end())
            {
                aggregateCost += found->second->cost;
            }
        }
    }

    return aggregateCost;
}

////////////////////////////////////////////////////////////////////////////////////////////////////
void Grid2d::calculateAggregateArea(int x0, int y0, int& xi, int& xf, int& yi, int& yf)
{
    xi = x0 - aggregateWidth_;
    xi = xi < 0 ? 0 : xi;

    xf = xi + aggregateWidth_ * 2;
    xf = xf >= width_ ? width_ - 1 : xf;

    yi = y0 - aggregateHeight_;
    yi = yi < 0 ? 0 : yi;

    yf = yi + aggregateHeight_ * 2;
    yf = yf >= height_ ? height_ - 1 : yf;
}

////////////////////////////////////////////////////////////////////////////////////////////////////
void Grid2d::updateAllAggregate()
{
    for (auto node : grid_)
    {
        node.second->aggregateCost = calculateAggregateCost(node.second);
    }
}

////////////////////////////////////////////////////////////////////////////////////////////////////
void Grid2d::updateNodeAggregate(Node* node, int oldCost)
{
    int xi;
    int xf;
    int yi;
    int yf;
    calculateAggregateArea(node->location.x, node->location.y, xi, xf, yi, yf);

    int deltaCost = BasicMath::noOverflowAdd<int>(node->cost, -oldCost);

    for (int y = yi; y <= yf; ++y)
    {
        for (int x = xi; x <= xf; ++x)
        {
            auto found = grid_.find(Location(x, y));
            if (found != grid_.end())
            {
                Node* neighbor = found->second;
                neighbor->aggregateCost = BasicMath::noOverflowAdd<int>(
                    neighbor->aggregateCost, deltaCost);
            }
        }
    }
}

} // namespace srs
