/*
 * (c) Copyright 2015-2016 River Systems, all rights reserved.
 *
 * This is proprietary software, unauthorized distribution is not permitted.
 */
#ifndef ROSTAPINTERNALGOALSOLUTION_HPP_
#define ROSTAPINTERNALGOALSOLUTION_HPP_

#include <ros/ros.h>

#include <srslib_framework/MsgSolution.h>

#include <srslib_framework/planning/pathplanning/Solution.hpp>

#include <srslib_framework/ros/tap/RosTap.hpp>
#include <srslib_framework/ros/message/SolutionMessageFactory.hpp>

namespace srs {

class RosTapInternal_GoalSolution :
    public RosTap
{
public:
    RosTapInternal_GoalSolution() :
        RosTap("/internal/state/goal/solution", "Goal Solution Tap")
    {}

    ~RosTapInternal_GoalSolution()
    {
        disconnectTap();
    }

    Solution<Grid2dSolutionItem> getSolution()
    {
        setNewData(false);
        return currentSolution_;
    }

    void reset()
    {
        RosTap::reset();

        Solution<Grid2dSolutionItem> solution;
        set(solution);
    }

    void set(Solution<Grid2dSolutionItem>& solution)
    {
        currentSolution_ = solution;
        setNewData(true);
    }

protected:
    bool connect()
    {
        rosSubscriber_ = rosNodeHandle_.subscribe(getTopic(), 10,
            &RosTapInternal_GoalSolution::onSolution, this);
        return true;
    }

private:
    void onSolution(const srslib_framework::MsgSolutionConstPtr message)
    {
        Solution<Grid2dSolutionItem> solution = SolutionMessageFactory::msg2Solution(message);
        set(solution);
    }

    Solution<Grid2dSolutionItem> currentSolution_;
};

} // namespace srs

#endif // ROSTAPINTERNALGOALSOLUTION_HPP_
