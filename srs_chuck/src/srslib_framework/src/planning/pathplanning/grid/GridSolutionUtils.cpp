#include <srslib_framework/planning/pathplanning/grid/GridSolutionUtils.hpp>

namespace srs {

////////////////////////////////////////////////////////////////////////////////////////////////////
// Public methods

////////////////////////////////////////////////////////////////////////////////////////////////////
double GridSolutionUtils::getTotalCost(Solution<GridSolutionItem>& solution)
{
    return solution.getGoal().cost;
}

////////////////////////////////////////////////////////////////////////////////////////////////////
vector<Solution<GridSolutionItem>*> GridSolutionUtils::splitSolution(
    Solution<GridSolutionItem>& solution)
{
    vector<Solution<GridSolutionItem>*> result;
    if (solution.empty())
    {
        return result;
    }

    Solution<GridSolutionItem>* currentSolution = new Solution<GridSolutionItem>();
    result.push_back(currentSolution);

    GridSolutionItem::ActionEnum currentAction = solution.getStart().actionType;

    for (auto solutionItem : solution)
    {
        if (solutionItem.actionType != currentAction)
        {
            currentSolution = new Solution<GridSolutionItem>();
            result.push_back(currentSolution);
        }

        currentSolution->push_back(solutionItem);
        currentAction = solutionItem.actionType;
    }

    return result;
}

} // namespace srs
