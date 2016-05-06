/*
 * (c) Copyright 2015-2016 River Systems, all rights reserved.
 *
 * This is proprietary software, unauthorized distribution is not permitted.
 */
#ifndef SEARCHACTION_HPP_
#define SEARCHACTION_HPP_

#include <array>
#include <string>
#include <unordered_map>
using namespace std;

#include <srslib_framework/math/Math.hpp>

#include <srslib_framework/search/SearchPosition.hpp>

namespace srs {

template<typename GRAPH>
struct SearchNode;

template<typename GRAPH>
struct SearchAction
{
    enum ActionEnum {NONE, FORWARD, BACKWARD, ROTATE_M90, ROTATE_P90, ROTATE_180};

    static array<ActionEnum, 5> ACTIONS;

    static SearchAction<GRAPH> instanceOf(ActionEnum action, GRAPH& graph, SearchNode<GRAPH>* current)
    {
        switch (action)
        {
            case FORWARD: return addForward(graph, current);
            case BACKWARD: return addBackward(graph, current);
            case ROTATE_M90: return addRotation(ROTATE_M90, current, -90);
            case ROTATE_P90: return addRotation(ROTATE_P90, current, 90);
            case ROTATE_180: return addRotation(ROTATE_180, current, 180);
        }

        return SearchAction();
    }

    SearchAction(ActionEnum action = NONE,
            SearchPosition<GRAPH> position = SearchPosition<GRAPH>(),
            int g = 0, int h = 0) :
        action(action),
        position(position),
        g(g),
        h(h)
    {}

    friend ostream& operator<<(ostream& stream, const SearchAction& searchAction)
    {
        return stream << searchAction.position << " " << ENUM_NAMES[searchAction.action];
    }

    ActionEnum action;
    SearchPosition<GRAPH> position;
    int g;
    int h;

private:

    static unordered_map<int, string> ENUM_NAMES;

    static SearchAction<GRAPH> addBackward(GRAPH& graph, SearchNode<GRAPH>* current)
    {
        SearchAction<GRAPH> action(BACKWARD);
        int currentOrientation = current->action.position.orientation;

        int direction = Math::normalizeAngleDeg(currentOrientation - 180);

        typename GRAPH::LocationType neighbor;
        if (graph.getNeighbor(current->action.position.location, direction, neighbor))
        {
            SearchPosition<GRAPH> newPosition(neighbor, currentOrientation);

            int commandCost = getCommandCost(BACKWARD);
            int locationCost = graph.getCost(neighbor);

            action.position = newPosition;
            action.g = current->action.g + locationCost + commandCost;
            action.h = current->action.h;

            return action;
        }

        return SearchAction<GRAPH>(NONE);
    }

    static SearchAction<GRAPH> addForward(GRAPH& graph, SearchNode<GRAPH>* current)
    {
        SearchAction<GRAPH> action(FORWARD);
        int currentOrientation = current->action.position.orientation;

        typename GRAPH::LocationType neighbor;
        if (graph.getNeighbor(current->action.position.location, currentOrientation, neighbor))
        {
            SearchPosition<GRAPH> newPosition(neighbor, currentOrientation);

            int commandCost = getCommandCost(FORWARD);
            int locationCost = graph.getCost(neighbor);

            action.position = newPosition;
            action.g = current->action.g + locationCost + commandCost;
            action.h = current->action.h;

            return action;
        }

        return SearchAction<GRAPH>(NONE);
    }

    static SearchAction<GRAPH> addRotation(ActionEnum actionEnum, SearchNode<GRAPH>* current, int angle)
    {
        SearchAction<GRAPH> action(actionEnum);
        int currentOrientation = current->action.position.orientation;

        int newOrientation = Math::normalizeAngleDeg(currentOrientation - angle);
        SearchPosition<GRAPH> newPosition(current->action.position, newOrientation);

        int commandCost = getCommandCost(actionEnum);

        action.position = newPosition;
        action.g = current->action.g + commandCost;
        action.h = current->action.h;

        return action;
    }

    static int getCommandCost(ActionEnum action)
    {
        switch (action)
        {
            case FORWARD: return 1;
            case BACKWARD: return 40;
            case ROTATE_M90: return 2;
            case ROTATE_P90: return 2;
            case ROTATE_180: return 4;
        }

        throw;
    }
};

template<typename GRAPH>
array<typename SearchAction<GRAPH>::ActionEnum, 5> SearchAction<GRAPH>::ACTIONS = {
    FORWARD,
    BACKWARD,
    ROTATE_M90,
    ROTATE_P90,
    ROTATE_180
};

template<typename GRAPH>
unordered_map<int, string> SearchAction<GRAPH>::ENUM_NAMES = {
    {SearchAction<GRAPH>::NONE, "NONE"},
    {SearchAction<GRAPH>::FORWARD, "FORWARD"},
    {SearchAction<GRAPH>::BACKWARD, "BACKWARD"},
    {SearchAction<GRAPH>::ROTATE_M90, "ROTATE_M90"},
    {SearchAction<GRAPH>::ROTATE_P90, "ROTATE_P90"},
    {SearchAction<GRAPH>::ROTATE_180, "ROTATE_180"}
};

} // namespace srs

#endif // SEARCHACTION_HPP_
