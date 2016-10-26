#include <srslib_framework/datastructure/graph/grid2d/Grid2d.hpp>

#include <srslib_framework/datastructure/graph/grid2d/OutOfRangeException.hpp>

namespace srs {

////////////////////////////////////////////////////////////////////////////////////////////////////
// Public methods

const Grid2d::BaseType Grid2d::PAYLOAD_MIN = 0;
const Grid2d::BaseType Grid2d::PAYLOAD_MAX = numeric_limits<Grid2d::BaseType>::max();
const Grid2d::BaseType Grid2d::WEIGHT_MIN = 0;
const Grid2d::BaseType Grid2d::WEIGHT_MAX = numeric_limits<Grid2d::BaseType>::max();

////////////////////////////////////////////////////////////////////////////////////////////////////
void Grid2d::clear()
{
    for (auto cell : grid_)
    {
        delete cell.second;
    }

    grid_.clear();
    weightCount_ = 0;
}

////////////////////////////////////////////////////////////////////////////////////////////////////
void Grid2d::clear(const Location& location)
{
    auto found = grid_.find(location);
    if (found != grid_.end())
    {
        if (found->second->weights)
        {
            weightCount_--;
        }
        delete found->second;
        grid_.erase(found);
    }
}

////////////////////////////////////////////////////////////////////////////////////////////////////
Grid2d::BaseType Grid2d::getAggregate(const Location& location) const
{
    Node* node = findNode(location);
    if (node)
    {
        return node->aggregate;
    }

    return PAYLOAD_MIN;
}

////////////////////////////////////////////////////////////////////////////////////////////////////
Grid2d::BaseType Grid2d::getPayload(const Location& location) const
{
    Node* node = findNode(location);
    if (node)
    {
        return node->payload;
    }

    return PAYLOAD_MIN;
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
Grid2d::BaseType Grid2d::getWeight(const Location& location, int orientation) const
{
    if (weightCount_)
    {
        Node* node = findNode(location);
        if (node && node->weights)
        {
            switch (orientation)
            {
                case ORIENTATION_EAST:
                    return node->weights->east;

                case ORIENTATION_NORTH:
                    return node->weights->north;

                case ORIENTATION_WEST:
                    return node->weights->west;

                case ORIENTATION_SOUTH:
                    return node->weights->south;

                default:
                    return PAYLOAD_MAX;
            }
        }
    }

    return WEIGHT_MIN;
}

////////////////////////////////////////////////////////////////////////////////////////////////////
void Grid2d::maxOnPayload(const Location& location, BaseType otherPayload)
{
    updatePayload(location, otherPayload,
        [] (BaseType oldPayload, BaseType newPayload) -> BaseType
        {
            return max(oldPayload, newPayload);
        }
    );
}

////////////////////////////////////////////////////////////////////////////////////////////////////
ostream& operator<<(ostream& stream, const Grid2d& grid)
{
    stream << "Grid2d {" << endl;

    stream << "(" << grid.height_ << "x" << grid.width_ << ")" << endl;

    grid.printGrid(stream, "Simplified",
        [] (Grid2d::Node* node) -> Grid2d::BaseType
        {
            return node->payload;
        }
    );

    grid.print(stream, "Payload",
        [] (Grid2d::Node* node) -> Grid2d::BaseType
        {
            return node->payload;
        }
    );

    if (grid.aggregate_)
    {
        grid.print(stream, "Aggregate",
            [] (Grid2d::Node* node) -> Grid2d::BaseType
            {
                return node->aggregate;
            }
        );
    }

    if (grid.weightCount_)
    {
        grid.print(stream, "North Weights",
            [] (Grid2d::Node* node) -> Grid2d::BaseType
            {
                return node->weights ? node->weights->north : Grid2d::WEIGHT_MIN;
            }
        );

        grid.print(stream, "East Weights",
            [] (Grid2d::Node* node) -> Grid2d::BaseType
            {
                return node->weights ? node->weights->east : Grid2d::WEIGHT_MIN;
            }
        );

        grid.print(stream, "South Weights",
            [] (Grid2d::Node* node) -> Grid2d::BaseType
            {
                return node->weights ? node->weights->south : Grid2d::WEIGHT_MIN;
            }
        );

        grid.print(stream, "West Weights",
            [] (Grid2d::Node* node) -> Grid2d::BaseType
            {
                return node->weights ? node->weights->west : Grid2d::WEIGHT_MIN;
            }
        );
    }

    stream << "}";

    return stream;
}

////////////////////////////////////////////////////////////////////////////////////////////////////
bool operator==(const Grid2d& lhs, const Grid2d& rhs)
{
    // First check addresses
    if (&lhs == &rhs)
    {
        return true;
    }

    // Check that the dimensions are equal
    if (lhs.width_ != rhs.width_)
    {
        return false;
    }
    if (lhs.height_ != rhs.height_)
    {
        return false;
    }

    // Check that the aggregate parameters are equal
    if (lhs.aggregate_ != rhs.aggregate_)
    {
        return false;
    }
    if (lhs.aggregateWidth_ != rhs.aggregateWidth_)
    {
        return false;
    }
    if (lhs.aggregateWidth_ != rhs.aggregateWidth_)
    {
        return false;
    }

    // Check that the grids contain the same number of items
    if (lhs.getOccupiedCount() != rhs.getOccupiedCount())
    {
        return false;
    }
    if (lhs.getWeightCount() != rhs.getWeightCount())
    {
        return false;
    }

    // Check that the grids contain the same items
    for (auto it : lhs.grid_)
    {
        Grid2d::Node* node = rhs.findNode(it.first);
        if (!node || *it.second != *node)
        {
            return false;
        }
    }

    return true;
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
void Grid2d::setPayload(const Location& location, BaseType newPayload)
{
    updatePayload(location, newPayload,
        [] (BaseType oldPayload, BaseType newPayload) -> BaseType
        {
            return newPayload;
        }
    );
}

////////////////////////////////////////////////////////////////////////////////////////////////////
void Grid2d::setWeights(const Location& location,
    BaseType north, BaseType east, BaseType south, BaseType west)
{
    Node* node = findNode(location);

    bool nonZeroWeights = north > WEIGHT_MIN || east > WEIGHT_MIN ||
        south > WEIGHT_MIN || west > WEIGHT_MIN;

    if (!node)
    {
        if (nonZeroWeights)
        {
            node = addNode(location, PAYLOAD_MIN, PAYLOAD_MIN);
        }
        else
        {
            return;
        }
    }

    if (!node->weights)
    {
        if (nonZeroWeights)
        {
            weightCount_++;
            node->weights = new Weights(north, east, south, west);
        }
    }
    else
    {
        if (nonZeroWeights)
        {
            node->weights->north = north;
            node->weights->east = east;
            node->weights->south = south;
            node->weights->west = west;
        }
        else
        {
            delete node->weights;
            node->weights = nullptr;
            weightCount_--;
        }
    }
}

////////////////////////////////////////////////////////////////////////////////////////////////////
// Private methods

////////////////////////////////////////////////////////////////////////////////////////////////////
int Grid2d::calculateAggregate(Node* node)
{
    int xi;
    int xf;
    int yi;
    int yf;
    calculateAggregateArea(node->location.x, node->location.y, xi, xf, yi, yf);

    BaseType aggregate = PAYLOAD_MIN;
    for (int y = yi; y <= yf; ++y)
    {
        for (int x = xi; x <= xf; ++x)
        {
            auto found = grid_.find(Location(x, y));
            if (found != grid_.end())
            {
                aggregate = BasicMath::noOverflowAdd<BaseType>(aggregate, found->second->payload);
            }
        }
    }

    return aggregate;
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
    std::function<BaseType (Node*)> fieldSelection) const
{
    if (!title.empty())
    {
        stream << endl << title << endl;
    }

    for (int y = height_ - 1; y >= 0; --y)
    {
        for (int x = 0; x < width_; ++x)
        {
            Grid2d::Node* node = findNode(Grid2d::Location(x, y));
            if (node)
            {
                BaseType payload = fieldSelection(node);

                if (payload == Grid2d::PAYLOAD_MAX)
                {
                    stream << "# ";
                }
                else if (payload == Grid2d::PAYLOAD_MIN)
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
void Grid2d::print(ostream& stream, string title,
    std::function<BaseType (Node*)> fieldSelection) const
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

    for (int y = height_ - 1; y >= 0; --y)
    {
        stream << right << setw(WIDTH) << y;
        for (int x = 0; x < width_; ++x)
        {
            Grid2d::Node* node = findNode(Grid2d::Location(x, y));
            if (node)
            {
                BaseType info = fieldSelection(node);

                if (info == Grid2d::PAYLOAD_MAX)
                {
                    stream << right << setw(WIDTH) << "#";
                }
                else if (info == Grid2d::PAYLOAD_MIN)
                {
                    stream << right << setw(WIDTH) << ".";
                }
                else
                {
                    stream << right << setw(WIDTH) << static_cast<int>(info);
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
        node.second->aggregate = calculateAggregate(node.second);
    }
}

////////////////////////////////////////////////////////////////////////////////////////////////////
void Grid2d::updateNodeAggregate(Node* node, BaseType oldPayload, BaseType newPayload)
{
    int xi;
    int xf;
    int yi;
    int yf;
    calculateAggregateArea(node->location.x, node->location.y, xi, xf, yi, yf);

    BaseType delta = BasicMath::noOverflowAdd<BaseType>(newPayload, -oldPayload);
    for (int y = yi; y <= yf; ++y)
    {
        for (int x = xi; x <= xf; ++x)
        {
            auto found = grid_.find(Location(x, y));
            if (found != grid_.end())
            {
                Node* neighbor = found->second;
                neighbor->aggregate = BasicMath::noOverflowAdd<BaseType>(neighbor->aggregate, delta);
            }
        }
    }
}

////////////////////////////////////////////////////////////////////////////////////////////////////
void Grid2d::updatePayload(const Location& location, BaseType newPayload,
    std::function<BaseType (BaseType, BaseType)> payloadSelection)
{
    if (!isWithinBounds(location))
    {
        throw OutOfRangeException(location, Location(width_, height_));
    }

    BaseType oldPayload = PAYLOAD_MIN;
    BaseType finalPayload = PAYLOAD_MIN;

    bool modifiedGrid = true;

    Node* node = findNode(location);
    if (node)
    {
        finalPayload = payloadSelection(node->payload, newPayload);
        oldPayload = node->payload;

        node->payload = finalPayload;
    }
    else
    {
        finalPayload = payloadSelection(PAYLOAD_MIN, newPayload);

        // Do not create a new node if the specified payload
        // is the minimum available payload
        if (finalPayload > PAYLOAD_MIN)
        {
            node = addNode(location, PAYLOAD_MIN, PAYLOAD_MIN);
            if (aggregate_)
            {
                node->aggregate = calculateAggregate(node);
            }
            node->payload = finalPayload;
        }
        else
        {
            modifiedGrid = false;
        }
    }

    // If the grid was modified and
    // the aggregate is calculate, refresh the neighbors
    if (modifiedGrid)
    {
        if (aggregate_)
        {
            updateNodeAggregate(node, oldPayload, finalPayload);
        }
        else
        {
            // If no aggregation is calculated, the property
            // follows the payload (in case aggregation is activated later)
            node->aggregate = node->payload;
        }

        // Prune the grid if there is no additional weight information and
        // the new cost of the node is the minimum
        if (finalPayload == PAYLOAD_MIN)
        {
            if (!node->weights)
            {
                clear(location);
            }
        }
    }
}

} // namespace srs
