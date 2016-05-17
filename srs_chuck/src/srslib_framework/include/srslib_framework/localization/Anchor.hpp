/*
 * (c) Copyright 2015-2016 River Systems, all rights reserved.
 *
 * This is proprietary software, unauthorized distribution is not permitted.
 */
#ifndef ANCHOR_HPP_
#define ANCHOR_HPP_

namespace srs {

using namespace std;

struct Anchor
{
public:
    Anchor() :
        id(""),
        x(0),
        y(0),
        z(0),
        orientation(0)
    {}

    Anchor(string id, unsigned int x, unsigned int y, unsigned int z, unsigned int orientation) :
        id(id),
        x(x),
        y(y),
        z(z),
        orientation(orientation)
    {}

    string id;
    unsigned int x;
    unsigned int y;
    unsigned int z;

    unsigned int orientation;
};

} // namespace srs

#endif // ANCHOR_HPP_
