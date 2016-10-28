/*
 * (c) Copyright 2015-2016 River Systems, all rights reserved.
 *
 * This is proprietary software, unauthorized distribution is not permitted.
 */
#pragma once

#include <vector>
#include <algorithm>

#include <srsnode_navigation/global_planner/PotentialCalculator.hpp>
#include <srsnode_navigation/global_planner/SrsPlannerPotentials.hpp>

namespace srs {

class Index {
    public:
        Index(int a, float b) {
            i = a;
            cost = b;
        }
        int i;
        float cost;
};

struct greater1 {
        bool operator()(const Index& a, const Index& b) const {
            return a.cost > b.cost;
        }
};

class AStarExpansion : public Expander
{
public:
    AStarExpansion(PotentialCalculator* p_calc, int nx, int ny);
    bool calculatePotentials(unsigned char* costs,
        double start_x, double start_y, double end_x, double end_y, int cycles,
        float* potential);

private:
    void add(unsigned char* costs, float* potential, float prev_potential, int next_i, int end_x, int end_y);
    std::vector<Index> queue_;
};

}
