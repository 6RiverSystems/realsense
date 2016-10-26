#include <srslib_framework/planning/pathplanning/grid2d/Grid2dSolutionUtils.hpp>

namespace srs {

////////////////////////////////////////////////////////////////////////////////////////////////////
// Public methods

////////////////////////////////////////////////////////////////////////////////////////////////////
int Grid2dSolutionUtils::getTotalCost(Solution<Grid2dSolutionItem>& solution)
{
    return solution.getGoal().cost;
}

////////////////////////////////////////////////////////////////////////////////////////////////////
vector<Solution<Grid2dSolutionItem>*> Grid2dSolutionUtils::splitSolution(
    Solution<Grid2dSolutionItem>& solution)
{
    vector<Solution<Grid2dSolutionItem>*> result;
    if (solution.empty())
    {
        return result;
    }

    Solution<Grid2dSolutionItem>* currentSolution = new Solution<Grid2dSolutionItem>();
    result.push_back(currentSolution);

    Grid2dSolutionItem::ActionEnum currentAction = solution.getStart().actionType;

    for (auto solutionItem : solution)
    {
        if (solutionItem.actionType != currentAction)
        {
            currentSolution = new Solution<Grid2dSolutionItem>();
            result.push_back(currentSolution);
        }

        currentSolution->push_back(solutionItem);
        currentAction = solutionItem.actionType;
    }

    return result;
}

} // namespace srs
