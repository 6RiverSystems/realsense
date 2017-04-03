#include <srsnode_navigation/global_planner/potentials/QuadraticCalculator.hpp>

namespace srs {

////////////////////////////////////////////////////////////////////////////////////////////////////
float QuadraticCalculator::calculatePotential(float* potentials, unsigned int cost,
    int n, float prev_potential)
{
    // get neighbors
    float u, d, l, r;
    l = potentials[n - 1];
    r = potentials[n + 1];
    u = potentials[n - nx_];
    d = potentials[n + nx_];

    // find lowest, and its lowest neighbor
    float ta, tc;
    if (l < r)
    {
        tc = l;
    }
    else
    {
        tc = r;
    }

    if (u < d)
    {
        ta = u;
    }
    else
    {
        ta = d;
    }

    float hf = cost; // traversability factor
    float dc = tc - ta;        // relative cost between ta,tc

    if (dc < 0)         // tc is lowest
    {
        dc = -dc;
        ta = tc;
    }

    // calculate new potential
    if (dc >= hf)
    {
        // if too large, use ta-only update
        return ta + hf;
    }
    else            // two-neighbor interpolation update
    {
        // use quadratic approximation
        // might speed this up through table lookup, but still have to
        //   do the divide
        float d = dc / hf;
        float v = -0.2301 * d * d + 0.5307 * d + 0.7040;
        return ta + hf * v;
    }
}

}