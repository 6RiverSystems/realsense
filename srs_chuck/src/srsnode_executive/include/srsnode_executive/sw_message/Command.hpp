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

class Executive;

class Command
{
public:
    static constexpr int POSITION_COMMAND = 0;
    static constexpr int POSITION_PARAM_1 = 1;
    static constexpr int POSITION_PARAM_2 = 2;

    Command(Executive* owner, int minExpectedArguments = 0) :
        owner_(owner),
        minExpectedArguments_(minExpectedArguments)
    {}

    virtual ~Command()
    {}

    virtual bool execute(const vector<string>& params) = 0;

    int getMinExpectedArguments() const
    {
        return minExpectedArguments_;
    }

    Executive* getOwner() const
    {
        return owner_;
    }

    bool param2Bool(const vector<string>& parameters, const int parameter)
    {
        std::string stringValue = parameters[parameter];

        return stringValue == "on" || stringValue == "ON" ||
            stringValue == "true" || stringValue == "TRUE";
    }

private:
    int minExpectedArguments_;

    Executive* owner_;
};

} // namespace srs
