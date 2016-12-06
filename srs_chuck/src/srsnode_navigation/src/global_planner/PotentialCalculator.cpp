#include <srsnode_navigation/global_planner/PotentialCalculator.hpp>

#include <algorithm>
using namespace std;

namespace srs {

const float PotentialCalculator::MAX_POTENTIAL = 1.0e10;

////////////////////////////////////////////////////////////////////////////////////////////////////
float PotentialCalculator::calculatePotential(float* potentials, unsigned int cost,
        int n, float prev_potential)
{
    if (prev_potential < 0)
    {
        float min_h = min(potentials[n - 1], potentials[n + 1] );
        float min_v = min(potentials[n - nx_], potentials[n + nx_]);

        prev_potential = min(min_h, min_v);
    }

    return prev_potential + cost;
}

} // namespace srs
