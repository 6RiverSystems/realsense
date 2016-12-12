/*
 * (c) Copyright 2015-2016 River Systems, all rights reserved.
 *
 * This is proprietary software, unauthorized distribution is not permitted.
 */
#ifndef BASICMATH_HPP_
#define BASICMATH_HPP_

#include <cmath>
#include <limits>
using namespace std;

namespace srs {

struct BasicMath
{
    template<typename TYPE = double>
    inline static TYPE euclidean(TYPE x1, TYPE y1, TYPE x2, TYPE y2)
    {
        TYPE deltaX = x2 - x1;
        TYPE deltaY = y2 - y1;
        return sqrt(deltaX * deltaX + deltaY * deltaY);
    }

    template<typename TYPE = double>
    inline static bool equal(TYPE a, TYPE b, TYPE threshold = 4 * numeric_limits<TYPE>::epsilon())
    {
        return abs(a - b) < threshold;
    }

    template<typename TYPE = double>
    constexpr inline static bool isNan(TYPE value)
    {
        return value != value;
    }

    template<typename TYPE = int>
    inline static TYPE noOverflowAdd(TYPE a, TYPE b)
    {
        if (b > 0)
        {
            if (a > numeric_limits<TYPE>::max() - b)
            {
                return numeric_limits<TYPE>::max();
            }
            return a + b;
        }
        else
        {
            if (a < numeric_limits<TYPE>::min() - b)
            {
                return numeric_limits<TYPE>::min();
            }
            return a + b;
        }
    }

    template <typename TYPE = double>
    inline static TYPE saturate(TYPE value, TYPE max, TYPE min)
    {
        if (value < min)
        {
            return min;
        }
        else if (value > max)
        {
            return max;
        }

        return value;
    }

    template <typename TYPE = double>
    constexpr inline static TYPE sgn(TYPE value)
    {
        return TYPE((TYPE(0) < value) - (value < TYPE(0)));
    }

    template <typename TYPE = double>
    inline static TYPE threshold(TYPE value, TYPE threshold, TYPE minValue = TYPE())
    {
        return abs(value) > threshold ? value : minValue;
    }
};

} // namespace srs

#endif // BASICMATH_HPP_
