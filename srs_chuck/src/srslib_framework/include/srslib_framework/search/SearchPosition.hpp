/*
 * (c) Copyright 2015-2016 River Systems, all rights reserved.
 *
 * This is proprietary software, unauthorized distribution is not permitted.
 */
#ifndef SEARCHPOSITION_HPP_
#define SEARCHPOSITION_HPP_

#include <functional>
using namespace std;

namespace srs {

template<typename GRAPH>
struct SearchPosition
{
    typedef typename GRAPH::LocationType LocationType;

    static int heuristic(SearchPosition<GRAPH> from, SearchPosition<GRAPH> to)
    {
        return abs(from.location.x - to.location.x) + abs(from.location.y - to.location.y);
    }

    SearchPosition(LocationType location = LocationType(), int orientation = 0) :
        location(location),
        orientation(orientation)
    {}

    SearchPosition(SearchPosition<GRAPH> position, int orientation) :
        location(position.location),
        orientation(orientation)
    {}

    friend bool operator==(const SearchPosition& lhs, const SearchPosition& rhs)
    {
        return (lhs.location == rhs.location) && (lhs.orientation == rhs.orientation);
    }

    friend ostream& operator<<(ostream& stream, const SearchPosition& searchPosition)
    {
        stream << "<" << searchPosition.location << ", " << searchPosition.orientation << ">";

        return stream;
    }

    LocationType location;
    int orientation;
};

} // namespace srs

namespace std {

// Hash definition for the SearchNode class
template<typename GRAPH>
struct hash<srs::SearchPosition<GRAPH>>
{
    unsigned long operator()(const srs::SearchPosition<GRAPH>& node) const
    {
        return hash<typename GRAPH::LocationType>()(node.location) + 101 * node.orientation;
    }
};

template<typename GRAPH>
struct equal_to<srs::SearchPosition<GRAPH>>
{
    bool operator()(const srs::SearchPosition<GRAPH>& lhs, const srs::SearchPosition<GRAPH>& rhs) const
    {
        return (lhs.location == rhs.location) && (lhs.orientation == rhs.orientation);
    }
};

} // namespace std

#endif // SEARCHPOSITION_HPP_
