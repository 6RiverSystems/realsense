/*
 * (c) Copyright 2015-2016 River Systems, all rights reserved.
 *
 * This is proprietary software, unauthorized distribution is not permitted.
 */
#pragma once

#include <srslib_framework/ros/unit/RosUnit.hpp>

#include <srsnode_executive/LabeledAreasDetector.hpp>

namespace srs {

class Executive : public RosUnit<Executive>
{
public:
    Executive(string name, int argc, char** argv);
    ~Executive()
    {}

protected:
    void execute();

    void initialize();

private:
    constexpr static double REFRESH_RATE_HZ = 5; // [Hz]

    void findActiveNodes(vector<string>& nodes);

    void updateRobotPose();

    LabeledAreasDetector labeledAreasDetector_;

    Pose<> robotPose_;

    tf::TransformListener tfListener_;
};

} // namespace srs
