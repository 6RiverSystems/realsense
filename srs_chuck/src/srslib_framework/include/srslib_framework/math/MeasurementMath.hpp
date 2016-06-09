/*
 * (c) Copyright 2015-2016 River Systems, all rights reserved.
 *
 * This is proprietary software, unauthorized distribution is not permitted.
 */
#ifndef MEASUREMENTMATH_HPP_
#define MEASUREMENTMATH_HPP_

#include <cmath>
#include <limits>
using namespace std;

namespace srs {

struct MeasurementMath
{
    template<typename TYPE = double>
    constexpr inline static TYPE inch2m(TYPE inch)
    {
        return TYPE(0.0254) * inch;
    }

    template<typename TYPE = double>
    constexpr inline static TYPE inch2mm(TYPE inch)
    {
        return TYPE(25.4) * inch;
    }

    template<typename TYPE = double>
    constexpr inline static TYPE mm2inch(TYPE mm)
    {
        return mm / TYPE(25.4);
    }
};

} // namespace srs

#endif // MEASUREMENTMATH_HPP_
