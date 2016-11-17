#include <srslib_framework/datastructure/graph/grid2d/Grid2d.hpp>

#include <srslib_framework/datastructure/graph/grid2d/OutOfRangeException.hpp>

namespace srs {

////////////////////////////////////////////////////////////////////////////////////////////////////
// Public methods

const Grid2d::BaseType Grid2d::PAYLOAD_MIN = 0;
const Grid2d::BaseType Grid2d::PAYLOAD_MAX = 254;
const Grid2d::BaseType Grid2d::PAYLOAD_NO_INFORMATION = 255;

const Grid2d::BaseType Grid2d::WEIGHT_MIN = 0;
const Grid2d::BaseType Grid2d::WEIGHT_MAX = 255;
const Grid2d::BaseType Grid2d::WEIGHT_NO_INFORMATION = 255;

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
Grid2d::BaseType Grid2d::getPayload(const Location& location) const
{
    Node* node = findNode(location);
    if (node)
    {
        return node->payload;
    }

    return PAYLOAD_NO_INFORMATION;
}

////////////////////////////////////////////////////////////////////////////////////////////////////
Grid2d::BaseType Grid2d::getPayload(const Position& position) const
{
    return getPayload(Grid2d::Location(position.x, position.y));
}

////////////////////////////////////////////////////////////////////////////////////////////////////
bool Grid2d::getNeighbor(const Position& position, Position& result) const
{
    switch (position.orientation)
    {
        case ORIENTATION_EAST:
            result = Position(position.x + 1, position.y, position.orientation);
            break;

        case ORIENTATION_NORTH:
            result = Position(position.x, position.y + 1, position.orientation);
            break;

        case ORIENTATION_WEST:
            result = Position(position.x - 1, position.y, position.orientation);
            break;

        case ORIENTATION_SOUTH:
            result = Position(position.x, position.y - 1, position.orientation);
            break;

        default:
            return false;
    }

    return isWithinBounds(result);
}

////////////////////////////////////////////////////////////////////////////////////////////////////
Grid2d::BaseType Grid2d::getWeight(const Position& position) const
{
    if (weightCount_)
    {
        Node* node = findNode(Grid2d::Location(position.x, position.y));
        if (node && node->weights)
        {
            switch (position.orientation)
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
                    return WEIGHT_NO_INFORMATION;
            }
        }
    }

    return WEIGHT_NO_INFORMATION;
}

////////////////////////////////////////////////////////////////////////////////////////////////////
void Grid2d::getWeights(const Location& location,
    BaseType& north, BaseType& east, BaseType& south, BaseType& west) const
{
    north = WEIGHT_NO_INFORMATION;
    east = WEIGHT_NO_INFORMATION;
    south = WEIGHT_NO_INFORMATION;
    west = WEIGHT_NO_INFORMATION;

    if (weightCount_)
    {
        Node* node = findNode(location);
        if (node && node->weights)
        {
            north = node->weights->north;
            east = node->weights->east;
            south = node->weights->south;
            west = node->weights->west;
        }
    }
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

    if (grid.weightCount_)
    {
        grid.print(stream, "North Weights",
            [] (Grid2d::Node* node) -> Grid2d::BaseType
            {
                return node->weights ? node->weights->north : Grid2d::WEIGHT_NO_INFORMATION;
            }
        );

        grid.print(stream, "East Weights",
            [] (Grid2d::Node* node) -> Grid2d::BaseType
            {
                return node->weights ? node->weights->east : Grid2d::WEIGHT_NO_INFORMATION;
            }
        );

        grid.print(stream, "South Weights",
            [] (Grid2d::Node* node) -> Grid2d::BaseType
            {
                return node->weights ? node->weights->south : Grid2d::WEIGHT_NO_INFORMATION;
            }
        );

        grid.print(stream, "West Weights",
            [] (Grid2d::Node* node) -> Grid2d::BaseType
            {
                return node->weights ? node->weights->west : Grid2d::WEIGHT_NO_INFORMATION;
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

    bool nonZeroWeights = north != WEIGHT_NO_INFORMATION ||
        east != WEIGHT_NO_INFORMATION ||
        south != WEIGHT_NO_INFORMATION ||
        west != WEIGHT_NO_INFORMATION;

    if (!node)
    {
        if (nonZeroWeights)
        {
            node = addNode(location, PAYLOAD_MIN);
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
                    if (node->weights)
                    {
                        stream << "' ";
                    }
                    else
                    {
                        stream << ". ";
                    }
                }
                else
                {
                    stream << "+ ";
                }
            }
            else
            {
                stream << "? ";
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

    const int WIDTH = 4;
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
                else if (info == Grid2d::PAYLOAD_NO_INFORMATION)
                {
                    stream << right << setw(WIDTH) << "?";
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
                stream << right << setw(WIDTH) << "?";
            }
        }
        stream << endl;
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

    BaseType finalPayload = PAYLOAD_MIN;

    bool modifiedGrid = true;

    Node* node = findNode(location);
    if (node)
    {
        finalPayload = payloadSelection(node->payload, newPayload);
        node->payload = finalPayload;
    }
    else
    {
        finalPayload = payloadSelection(PAYLOAD_MIN, newPayload);

        // Do not create a new node if the specified payload
        // is the minimum available payload
        if (finalPayload != PAYLOAD_NO_INFORMATION)
        {
            node = addNode(location, finalPayload);
        }
        else
        {
            modifiedGrid = false;
        }
    }

    if (modifiedGrid)
    {
        // Prune the grid if there is no additional weight information and
        // the new cost of the node is the minimum
        if (finalPayload == PAYLOAD_NO_INFORMATION)
        {
            if (!node->weights)
            {
                clear(location);
            }
        }
    }
}

} // namespace srs
