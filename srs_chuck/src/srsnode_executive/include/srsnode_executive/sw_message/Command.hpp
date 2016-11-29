/*
 * (c) Copyright 2015-2016 River Systems, all rights reserved.
 *
 * This is proprietary software, unauthorized distribution is not permitted.
 */
#pragma once

#include <string>
#include <vector>
using namespace std;

#include <ros/ros.h>

namespace srs {

class Command
{
public:
    static constexpr int COMMAND = 0;
    static constexpr int PARAM_1 = 1;
    static constexpr int PARAM_2 = 2;

    Command(int minExpectedArguments = 0) :
        minExpectedArguments_(minExpectedArguments)
    {}

    virtual ~Command()
    {}

    virtual bool execute(const vector<string>& params) = 0;

    int getMinExpectedArguments() const
    {
        return minExpectedArguments_;
    }

    bool getSwitchParam(const vector<string>& params, const int parameter)
    {
        std::string switchParam = params[parameter];

        return switchParam == "ON" || switchParam == "on";
    }

private:
    int minExpectedArguments_;
};

} // namespace srs
