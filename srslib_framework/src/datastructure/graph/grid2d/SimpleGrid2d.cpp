#include <srslib_framework/datastructure/graph/grid2d/SimpleGrid2d.hpp>

#include <srslib_framework/datastructure/graph/grid2d/OutOfRangeException.hpp>

namespace srs {

////////////////////////////////////////////////////////////////////////////////////////////////////
// Public methods

const SimpleGrid2d::BaseType SimpleGrid2d::PAYLOAD_MIN = 0;
const SimpleGrid2d::BaseType SimpleGrid2d::PAYLOAD_MAX = 254;
const SimpleGrid2d::BaseType SimpleGrid2d::PAYLOAD_NO_INFORMATION = 255;

////////////////////////////////////////////////////////////////////////////////////////////////////
SimpleGrid2d::SimpleGrid2d(const SimpleGrid2d& other) :
    BaseGrid2d(other.getWidth(), other.getHeight())
{
    for (auto node : other.grid_)
    {
        Node* internalNode = node.second;
        addNode(internalNode->location, internalNode->payload);
    }
}

////////////////////////////////////////////////////////////////////////////////////////////////////
void SimpleGrid2d::clear()
{
    for (auto node : grid_)
    {
        delete node.second;
    }

    grid_.clear();
}

////////////////////////////////////////////////////////////////////////////////////////////////////
void SimpleGrid2d::clear(const Location& location)
{
    auto found = grid_.find(location);
    if (found != grid_.end())
    {
        delete found->second;
        grid_.erase(found);
    }
}

////////////////////////////////////////////////////////////////////////////////////////////////////
SimpleGrid2d::BaseType SimpleGrid2d::getPayload(const Location& location) const
{
    Node* node = findNode(location);
    if (node)
    {
        return node->payload;
    }

    return PAYLOAD_NO_INFORMATION;
}

////////////////////////////////////////////////////////////////////////////////////////////////////
SimpleGrid2d::BaseType SimpleGrid2d::getPayload(const Position& position) const
{
    Node* node = findNode(position);
    if (node)
    {
        return node->payload;
    }

    return PAYLOAD_NO_INFORMATION;
}

////////////////////////////////////////////////////////////////////////////////////////////////////
void SimpleGrid2d::payloadMax(const Location& location, BaseType otherPayload)
{
    updatePayload(location, otherPayload,
        [] (BaseType oldPayload, BaseType newPayload) -> BaseType
        {
            return max(oldPayload, newPayload);
        }
    );
}

////////////////////////////////////////////////////////////////////////////////////////////////////
void SimpleGrid2d::payloadSet(const Location& location, BaseType newPayload)
{
    updatePayload(location, newPayload,
        [] (BaseType oldPayload, BaseType newPayload) -> BaseType
        {
            return newPayload;
        }
    );
}

////////////////////////////////////////////////////////////////////////////////////////////////////
ostream& operator<<(ostream& stream, const SimpleGrid2d& grid)
{
    stream << "SimpleGrid2d {" << endl;

    stream << "(" << grid.getHeight() << "x" << grid.getWidth() << ")" << endl;

    grid.printGrid(stream, "Simplified",
        [] (SimpleGrid2d::Node* node) -> SimpleGrid2d::BaseType
        {
            return node->payload;
        }
    );

    grid.print(stream, "Payload",
        [] (SimpleGrid2d::Node* node) -> SimpleGrid2d::BaseType
        {
            return node->payload;
        }
    );

    stream << "}";

    return stream;
}

////////////////////////////////////////////////////////////////////////////////////////////////////
bool operator==(const SimpleGrid2d& lhs, const SimpleGrid2d& rhs)
{
    // First check addresses
    if (&lhs == &rhs)
    {
        return true;
    }

    // Check that the dimensions are equal
    if (lhs.getWidth() != rhs.getWidth())
    {
        return false;
    }
    if (lhs.getHeight() != rhs.getHeight())
    {
        return false;
    }

    // Check that the grids contain the same number of items
    if (lhs.getOccupiedCount() != rhs.getOccupiedCount())
    {
        return false;
    }

    // Check that the grids contain the same items
    for (auto it : lhs.grid_)
    {
        SimpleGrid2d::Node* node = rhs.findNode(it.first);
        if (!node || *it.second != *node)
        {
            return false;
        }
    }

    return true;
}

////////////////////////////////////////////////////////////////////////////////////////////////////
// Private methods

////////////////////////////////////////////////////////////////////////////////////////////////////
void SimpleGrid2d::printGrid(ostream& stream, string title,
    std::function<BaseType (Node*)> fieldSelection) const
{
    if (!title.empty())
    {
        stream << endl << title << endl;
    }

    size_t maxHeight = getHeight() - 1;
    for (size_t y = 0; y < getHeight(); ++y)
    {
        for (size_t x = 0; x < getWidth(); ++x)
        {
            SimpleGrid2d::Node* node = findNode(x, maxHeight-y);
            if (node)
            {
                BaseType field = fieldSelection(node);

                if (field == SimpleGrid2d::PAYLOAD_MAX)
                {
                    stream << "# ";
                }
                else if (field == SimpleGrid2d::PAYLOAD_MIN)
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
                stream << "? ";
            }
        }
        stream << endl;
    }
}

////////////////////////////////////////////////////////////////////////////////////////////////////
void SimpleGrid2d::print(ostream& stream, string title,
    std::function<BaseType (Node*)> fieldSelection) const
{
    if (!title.empty())
    {
        stream << endl << title << endl;
    }

    const int WIDTH = 4;
    stream << right << setw(WIDTH) << ' ';
    for (size_t x = 0; x < getWidth(); ++x)
    {
        stream << right << setw(WIDTH) << x;
    }
    stream << endl;

    size_t maxHeight = getHeight() - 1;
    for (size_t y = 0; y < getHeight(); ++y)
    {
        stream << right << setw(WIDTH) << y;
        for (size_t x = 0; x < getWidth(); ++x)
        {
            SimpleGrid2d::Node* node = findNode(x, maxHeight-y);
            if (node)
            {
                BaseType field = fieldSelection(node);

                if (field == SimpleGrid2d::PAYLOAD_MAX)
                {
                    stream << right << setw(WIDTH) << "#";
                }
                else if (field == SimpleGrid2d::PAYLOAD_NO_INFORMATION)
                {
                    stream << right << setw(WIDTH) << "?";
                }
                else if (field == SimpleGrid2d::PAYLOAD_MIN)
                {
                    stream << right << setw(WIDTH) << ".";
                }
                else
                {
                    stream << right << setw(WIDTH) << static_cast<int>(field);
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
void SimpleGrid2d::updatePayload(const Location& location, BaseType newPayload,
    std::function<BaseType (BaseType, BaseType)> payloadSelection)
{
    if (!isWithinBounds(location))
    {
        throw OutOfRangeException(location, Location(getWidth(), getHeight()));
    }

    BaseType finalPayload;
    Node* node = findNode(location);
    if (node)
    {
        finalPayload = payloadSelection(node->payload, newPayload);
        node->payload = finalPayload;

        // Remove the node if the specified payload has no information
        if (finalPayload == PAYLOAD_NO_INFORMATION)
        {
            clear(location);
        }
    }
    else
    {
        finalPayload = payloadSelection(PAYLOAD_MIN, newPayload);

        // Add a new node only if the specified payload
        // is not the minimum available payload
        if (finalPayload != PAYLOAD_NO_INFORMATION)
        {
            addNode(location, finalPayload);
        }
    }
}

} // namespace srs
