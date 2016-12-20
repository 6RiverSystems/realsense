#include <srsnode_navigation/global_planner/potentials/GridPath.hpp>

#include <algorithm>
#include <cstring>
#include <stdio.h>

#include <ros/ros.h>

#include <srsnode_navigation/global_planner/SrsPlannerPotentials.hpp>

namespace srs {

////////////////////////////////////////////////////////////////////////////////////////////////////
bool GridPath::getPath(float* potential,
    double start_x, double start_y, double goal_x, double goal_y,
    std::vector<std::pair<float, float> >& path)
{
    std::pair<float, float> current;
    current.first = goal_x;
    current.second = goal_y;

    int start_index = getIndex(start_x, start_y);

    path.push_back(current);
    int c = 0;
    int ns = xs_ * ys_;

    while (getIndex(current.first, current.second) != start_index)
    {
        float min_val = 1e10;
        int min_x = 0, min_y = 0;

//        if (checkP(current.first, current.second + 1, potential, min_val))
//        {
//            min_x = current.first;
//            min_y = current.second + 1;
//        }
//        if (checkP(current.first, current.second - 1, potential, min_val))
//        {
//            min_x = current.first;
//            min_y = current.second - 1;
//        }
//        if (checkP(current.first - 1, current.second, potential, min_val))
//        {
//            min_x = current.first - 1;
//            min_y = current.second;
//        }
//        if (checkP(current.first + 1, current.second, potential, min_val))
//        {
//            min_x = current.first + 1;
//            min_y = current.second;
//        }

        for (int xd = -1; xd <= 1; xd++)
        {
            for (int yd = -1; yd <= 1; yd++)
            {
                if (xd == 0 && yd == 0)
                {
                    continue;
                }

                int x = current.first + xd, y = current.second + yd;
                int index = getIndex(x, y);

                if (potential[index] < min_val)
                {
                    min_val = potential[index];
                    min_x = x;
                    min_y = y;
                }
            }
        }

        if (min_x == 0 && min_y == 0)
        {
            return false;
        }

        current.first = min_x;
        current.second = min_y;
        path.push_back(current);

        if (c++>ns*4)
        {
            return false;
        }

    }

    return true;
}

}
