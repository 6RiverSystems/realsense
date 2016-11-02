#include <srsnode_navigation/global_planner/GradientPath.hpp>
#include <srsnode_navigation/global_planner/SrsPlannerPotentials.hpp>

#include <ros/ros.h>
#include <algorithm>
#include <cstring>
#include <stdio.h>

//#include <global_planner/gradient_path.h>
//#include <global_planner/planner_core.h>

namespace srs {

////////////////////////////////////////////////////////////////////////////////////////////////////
GradientPath::GradientPath(PotentialCalculator* p_calc) :
        Traceback(p_calc), pathStep_(0.5) {
    gradx_ = grady_ = NULL;
}

////////////////////////////////////////////////////////////////////////////////////////////////////
GradientPath::~GradientPath()
{
    if (gradx_)
        delete[] gradx_;

    if (grady_)
        delete[] grady_;
}

////////////////////////////////////////////////////////////////////////////////////////////////////
void GradientPath::setSize(int xs, int ys)
{
    Traceback::setSize(xs, ys);

    if (gradx_)
        delete[] gradx_;

    if (grady_)
        delete[] grady_;

    gradx_ = new float[xs * ys];
    grady_ = new float[xs * ys];
}

////////////////////////////////////////////////////////////////////////////////////////////////////
bool GradientPath::getPath(float* potential,
    double start_x, double start_y, double goal_x, double goal_y,
    std::vector<std::pair<float, float> >& path)
{
    std::pair<float, float> current;
    int stc = getIndex(goal_x, goal_y);

    // set up offset
    float dx = goal_x - (int)goal_x;
    float dy = goal_y - (int)goal_y;
    int ns = xs_ * ys_;
    memset(gradx_, 0, ns * sizeof(float));
    memset(grady_, 0, ns * sizeof(float));

    int c = 0;
    while (c++ < ns*4)
    {
        // check if near goal
        double nx = stc % xs_ + dx, ny = stc / xs_ + dy;

        if (fabs(nx - start_x) < .5 && fabs(ny - start_y) < .5)
        {
            current.first = start_x;
            current.second = start_y;
            path.push_back(current);
            return true;
        }

        if (stc < xs_ || stc > xs_ * ys_ - xs_) // would be out of bounds
        {
            cout << "[PathCalc] Out of bounds" << endl;
            return false;
        }

        current.first = nx;
        current.second = ny;

        path.push_back(current);

        bool oscillation_detected = false;
        int npath = path.size();
        if (npath > 2 && path[npath - 1].first == path[npath - 3].first
                && path[npath - 1].second == path[npath - 3].second)
        {
            cout << "[PathCalc] oscillation detected, attempting fix" << endl;
            oscillation_detected = true;
        }

        int stcnx = stc + xs_;
        int stcpx = stc - xs_;

        // check for potentials at eight positions near cell
        if (potential[stc] >= POT_HIGH ||
            potential[stc + 1] >= POT_HIGH ||
            potential[stc - 1] >= POT_HIGH ||
            potential[stcnx] >= POT_HIGH ||
            potential[stcnx + 1] >= POT_HIGH ||
            potential[stcnx - 1] >= POT_HIGH ||
            potential[stcpx] >= POT_HIGH ||
            potential[stcpx + 1] >= POT_HIGH ||
            potential[stcpx - 1] >= POT_HIGH ||
            oscillation_detected)
        {
            cout << "[Path] Pot fn boundary, following grid ()" <<
                potential[stc] << " " << (int) path.size() << endl;

            // check eight neighbors to find the lowest
            int minc = stc;
            int minp = potential[stc];
            int st = stcpx - 1;
            if (potential[st] < minp)
            {
                minp = potential[st];
                minc = st;
            }
            st++;
            if (potential[st] < minp)
            {
                minp = potential[st];
                minc = st;
            }
            st++;
            if (potential[st] < minp)
            {
                minp = potential[st];
                minc = st;
            }
            st = stc - 1;
            if (potential[st] < minp)
            {
                minp = potential[st];
                minc = st;
            }
            st = stc + 1;
            if (potential[st] < minp)
            {
                minp = potential[st];
                minc = st;
            }
            st = stcnx - 1;
            if (potential[st] < minp)
            {
                minp = potential[st];
                minc = st;
            }
            st++;
            if (potential[st] < minp)
            {
                minp = potential[st];
                minc = st;
            }
            st++;
            if (potential[st] < minp)
            {
                minp = potential[st];
                minc = st;
            }
            stc = minc;
            dx = 0;
            dy = 0;

            if (potential[stc] >= POT_HIGH)
            {
                cout << "[PathCalc] No path found, high potential" << endl;
                return 0;
            }
        }

        // have a good gradient here
        else {

            // get grad at four positions near cell
            gradCell(potential, stc);
            gradCell(potential, stc + 1);
            gradCell(potential, stcnx);
            gradCell(potential, stcnx + 1);

            // get interpolated gradient
            float x1 = (1.0 - dx) * gradx_[stc] + dx * gradx_[stc + 1];
            float x2 = (1.0 - dx) * gradx_[stcnx] + dx * gradx_[stcnx + 1];
            float x = (1.0 - dy) * x1 + dy * x2; // interpolated x
            float y1 = (1.0 - dx) * grady_[stc] + dx * grady_[stc + 1];
            float y2 = (1.0 - dx) * grady_[stcnx] + dx * grady_[stcnx + 1];
            float y = (1.0 - dy) * y1 + dy * y2; // interpolated y

            // check for zero gradient, failed
            if (x == 0.0 && y == 0.0) {
                cout << "[PathCalc] Zero gradient" << endl;
                return false;
            }

            // move in the right direction
            float ss = pathStep_ / hypot(x, y);
            dx += x * ss;
            dy += y * ss;

            // check for overflow
            if (dx > 1.0) {
                stc++;
                dx -= 1.0;
            }
            if (dx < -1.0) {
                stc--;
                dx += 1.0;
            }
            if (dy > 1.0) {
                stc += xs_;
                dy -= 1.0;
            }
            if (dy < -1.0) {
                stc -= xs_;
                dy += 1.0;
            }

        }
    }

    return false;
}

////////////////////////////////////////////////////////////////////////////////////////////////////
float GradientPath::gradCell(float* potential, int n)
{
    if (gradx_[n] + grady_[n] > 0.0)    // check this cell
        return 1.0;

    if (n < xs_ || n > xs_ * ys_ - xs_)    // would be out of bounds
        return 0.0;
    float cv = potential[n];
    float dx = 0.0;
    float dy = 0.0;

    // check for in an obstacle
    if (cv >= POT_HIGH) {
        if (potential[n - 1] < POT_HIGH)
            dx = -lethal_cost_;
        else if (potential[n + 1] < POT_HIGH)
            dx = lethal_cost_;

        if (potential[n - xs_] < POT_HIGH)
            dy = -lethal_cost_;
        else if (potential[xs_ + 1] < POT_HIGH)
            dy = lethal_cost_;
    }

    else                // not in an obstacle
    {
        // dx calc, average to sides
        if (potential[n - 1] < POT_HIGH)
            dx += potential[n - 1] - cv;
        if (potential[n + 1] < POT_HIGH)
            dx += cv - potential[n + 1];

        // dy calc, average to sides
        if (potential[n - xs_] < POT_HIGH)
            dy += potential[n - xs_] - cv;
        if (potential[n + xs_] < POT_HIGH)
            dy += cv - potential[n + xs_];
    }

    // normalize
    float norm = hypot(dx, dy);
    if (norm > 0) {
        norm = 1.0 / norm;
        gradx_[n] = norm * dx;
        grady_[n] = norm * dy;
    }
    return norm;
}

}
