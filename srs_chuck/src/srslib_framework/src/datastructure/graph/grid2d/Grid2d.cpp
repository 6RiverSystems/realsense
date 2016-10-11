#include <srslib_framework/datastructure/graph/grid2d/Grid2d.hpp>

namespace srs {

////////////////////////////////////////////////////////////////////////////////////////////////////
// Public methods

const int Grid2d::COST_MIN = 0;
const int Grid2d::COST_MAX = numeric_limits<int>::max();

const int Grid2d::ORIENTATION_NORTH = 0;
const int Grid2d::ORIENTATION_EAST = 1;
const int Grid2d::ORIENTATION_SOUTH = 2;
const int Grid2d::ORIENTATION_WEST = 3;

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
void Grid2d::clear(const Location& location)
{
    auto found = grid_.find(location);
    if (found != grid_.end())
    {
        delete found->second;
        grid_.erase(found);
    }
}

////////////////////////////////////////////////////////////////////////////////////////////////////
int Grid2d::getAggregateCost(const Location& location) const
{
    Node* node = findNode(location);
    if (node)
    {
        return node->aggregateCost;
    }

    return COST_MIN;
}

////////////////////////////////////////////////////////////////////////////////////////////////////
int Grid2d::getCost(const Location& location) const
{
    Node* node = findNode(location);
    if (node)
    {
        return node->cost;
    }

    return COST_MIN;
}

////////////////////////////////////////////////////////////////////////////////////////////////////
bool Grid2d::getNeighbor(const Location& location, int orientation, Location& result) const
{
    switch (orientation)
    {
        case ORIENTATION_EAST:
            result = Location(location.x + 1, location.y);
            break;

        case ORIENTATION_NORTH:
            result = Location(location.x, location.y + 1);
            break;

        case ORIENTATION_WEST:
            result = Location(location.x - 1, location.y);
            break;

        case ORIENTATION_SOUTH:
            result = Location(location.x, location.y - 1);
            break;
    }

    return isWithinBounds(result);
}

////////////////////////////////////////////////////////////////////////////////////////////////////
int Grid2d::getWeight(const Location& location, int orientation) const
{
    if (hasWeights_)
    {
        Node* node = findNode(location);
        if (node && node->weights)
        {
            switch (orientation)
            {
                case ORIENTATION_EAST:
                case ORIENTATION_NORTH:
                case ORIENTATION_WEST:
                case ORIENTATION_SOUTH:
                    return node->weights->cost[orientation];

                default:
                    return COST_MAX;
            }
        }
    }

    return COST_MIN;
}

////////////////////////////////////////////////////////////////////////////////////////////////////
void Grid2d::maxCost(const Location& location, int cost)
{
    int oldCost = COST_MIN;

    Node* node = findNode(location);
    if (node)
    {
        oldCost = node->cost;
        node->cost = max(node->cost, cost);
    }
    else
    {
        node = addNode(location, cost, COST_MIN);
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
ostream& operator<<(ostream& stream, const Grid2d& grid)
{
    stream << "Grid2d {" << endl;

    stream << "(" << grid.height_ << "x" << grid.width_ << ")" << endl;

    grid.printGrid(stream, "Simplified",
        [] (Grid2d::Node* node) -> int
        {
            return node->cost;
        }
    );

    grid.printCosts(stream, "Cost",
        [] (Grid2d::Node* node) -> int
        {
            return node->cost;
        }
    );

    if (grid.aggregate_)
    {
        grid.printCosts(stream, "Aggregate",
            [] (Grid2d::Node* node) -> int
            {
                return node->aggregateCost;
            }
        );
    }

    if (grid.hasWeights_)
    {
        grid.printCosts(stream, "North Weights",
            [] (Grid2d::Node* node) -> int
            {
                return node->weights ?
                    node->weights->cost[Grid2d::ORIENTATION_NORTH] :
                    Grid2d::COST_MIN;
            }
        );

        grid.printCosts(stream, "East Weights",
            [] (Grid2d::Node* node) -> int
            {
                return node->weights ?
                    node->weights->cost[Grid2d::ORIENTATION_EAST] :
                    Grid2d::COST_MIN;
            }
        );

        grid.printCosts(stream, "South Weights",
            [] (Grid2d::Node* node) -> int
            {
                return node->weights ?
                    node->weights->cost[Grid2d::ORIENTATION_SOUTH] :
                    Grid2d::COST_MIN;
            }
        );

        grid.printCosts(stream, "West Weights",
            [] (Grid2d::Node* node) -> int
            {
                return node->weights ?
                    node->weights->cost[Grid2d::ORIENTATION_WEST] :
                    Grid2d::COST_MIN;
            }
        );
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
    int oldCost = COST_MIN;

    Node* node = findNode(location);
    if (node)
    {
        oldCost = node->cost;
        node->cost = newCost;
    }
    else
    {
        node = addNode(location, newCost, COST_MIN);
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
void Grid2d::setWeights(const Location& location, int north, int east, int south, int west)
{
    if (north == COST_MIN && east == COST_MIN && south == COST_MIN && west == COST_MIN)
    {
        return;
    }

    hasWeights_ = true;

    Node* node = findNode(location);
    if (!node)
    {
        node = addNode(location, COST_MIN, COST_MIN);
    }

    node->weights = new Weights(north, east, south, west);
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

    int aggregateCost = COST_MIN;
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
void Grid2d::printGrid(ostream& stream, string title,
    std::function<int (Node*)> fieldSelection) const
{
    if (!title.empty())
    {
        stream << endl << title << endl;
    }

    for (int y = 0; y < height_; ++y)
    {
        for (int x = 0; x < width_; ++x)
        {
            Grid2d::Node* node = findNode(Grid2d::Location(x, y));
            if (node)
            {
                int cost = fieldSelection(node);

                if (cost == Grid2d::COST_MAX)
                {
                    stream << "# ";
                }
                else if (cost == Grid2d::COST_MIN)
                {
                    stream << ". ";
                }
                else
                {
                    stream << "+ ";
                }
            }
            else
            {
                stream << ". ";
            }
        }
        stream << endl;
    }
}

////////////////////////////////////////////////////////////////////////////////////////////////////
void Grid2d::printCosts(ostream& stream, string title,
    std::function<int (Node*)> fieldSelection) const
{
    if (!title.empty())
    {
        stream << endl << title << endl;
    }

    stream << right << setw(WIDTH) << ' ';
    for (int x = 0; x < width_; ++x)
    {
        stream << right << setw(WIDTH) << x;
    }
    stream << endl;

    for (int y = 0; y < height_; ++y)
    {
        stream << right << setw(WIDTH) << y;
        for (int x = 0; x < width_; ++x)
        {
            Grid2d::Node* node = findNode(Grid2d::Location(x, y));
            if (node)
            {
                int cost = fieldSelection(node);

                if (cost == Grid2d::COST_MAX)
                {
                    stream << right << setw(WIDTH) << "#";
                }
                else if (cost == Grid2d::COST_MIN)
                {
                    stream << right << setw(WIDTH) << ".";
                }
                else
                {
                    stream << right << setw(WIDTH) << cost;
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
