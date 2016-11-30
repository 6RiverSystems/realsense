/*
 * (c) Copyright 2015-2016 River Systems, all rights reserved.
 *
 * This is proprietary software, unauthorized distribution is not permitted.
 */
#pragma once

#include <srslib_framework/exception/SrsRuntimeErrorException.hpp>
#include <srslib_framework/search/AStar.hpp>

namespace srs {

class UnexpectedSearchActionException: public SrsRuntimeErrorException
{
public:
    UnexpectedSearchActionException(AStar::SolutionType& solution) :
        SrsRuntimeErrorException("Unexpected valid search action in this solution " +
            toString(solution))
    {}

private:
    string toString(AStar::SolutionType& solution)
    {
        stringstream stream;

        int counter = 0;

        stream << "{" << endl;
        for (auto node : solution)
        {
            stream << setw(4) << counter++ << ": " << node << endl;
        }

        stream << "}";

        return stream.str();
    }
};

} // namespace srs
