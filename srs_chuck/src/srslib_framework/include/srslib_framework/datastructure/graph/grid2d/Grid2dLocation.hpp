/*
 * (c) Copyright 2015-2016 River Systems, all rights reserved.
 *
 * This is proprietary software, unauthorized distribution is not permitted.
 */
#ifndef GRID2DLOCATION_HPP_
#define GRID2DLOCATION_HPP_


namespace srs {

struct Grid2dLocation
{
    Grid2dLocation(int x = 0, int y = 0) :
        x(x),
        y(y)
    {}

    friend bool operator==(const Grid2dLocation& lhs, const Grid2dLocation& rhs)
    {
        return (lhs.x == rhs.x) && (lhs.y == rhs.y);
    }

    friend ostream& operator<<(ostream& stream, const Grid2dLocation& location)
    {
        return stream << location.x << ", " << location.y;
    }

    int x;
    int y;
};

} // namespace srs

namespace std {

template<>
struct hash<srs::Grid2dLocation>
{
    unsigned long operator()(const srs::Grid2dLocation& location) const
    {
        return location.x + 1812433253 * location.y;
    }
};

template<>
struct equal_to<srs::Grid2dLocation>
{
    bool operator()(const srs::Grid2dLocation& lhs, const srs::Grid2dLocation& rhs) const
    {
        return (lhs.x == rhs.x) && (lhs.y == rhs.y);
    }
};

}

#endif // GRID2DLOCATION_HPP_
