#include <srsnode_navigation/global_planner/PotentialCalculator.hpp>

namespace srs {

////////////////////////////////////////////////////////////////////////////////////////////////////
float PotentialCalculator::calculatePotential(float* potential,
        unsigned char cost,
        int n,
        float prev_potential)
    {
        if (prev_potential < 0)
        {
            float min_h = std::min( potential[n - 1], potential[n + 1] );
            float min_v = std::min( potential[n - nx_], potential[n + nx_]);

            prev_potential = std::min(min_h, min_v);
        }

        return prev_potential + cost;
    }

}
