/*
 * (c) Copyright 2015-2016 River Systems, all rights reserved.
 *
 * This is proprietary software, unauthorized distribution is not permitted.
 */
#ifndef COSTMAP2D_HPP_
#define COSTMAP2D_HPP_

#include <srslib_framework/graph/grid2d/Grid2d.hpp>

namespace srs {

class CostMap2d
{
public:
    CostMap2d();
    ~CostMap2d();

private:
    Grid2d* costGrid_;
};

} // namespace srs

#endif // COSTMAP2D_HPP_
