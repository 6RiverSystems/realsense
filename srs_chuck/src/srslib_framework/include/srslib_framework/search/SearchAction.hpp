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
#include <limits>
using namespace std;

#include <srslib_framework/math/Math.hpp>

#include <srslib_framework/search/SearchPosition.hpp>
#include <srslib_framework/search/SearchPositionNote.hpp>

namespace srs {

template<typename GRAPH>
struct SearchNode;

template<typename GRAPH>
struct SearchAction
{
    constexpr static unsigned int MAX_COST = numeric_limits<unsigned int>::max();

    enum ActionEnum {NONE, START, GOAL, FORWARD, BACKWARD, ROTATE_M90, ROTATE_P90, ROTATE_180};

    static array<ActionEnum, 5> ACTIONS;

    static SearchAction<GRAPH> instanceOf(ActionEnum action,
        GRAPH* graph, SearchNode<GRAPH>* associatedNode)
    {
        switch (action)
        {
            case FORWARD: return addForward(graph, associatedNode);
            case BACKWARD: return addBackward(graph, associatedNode);
            case ROTATE_M90: return addRotation(graph, associatedNode, ROTATE_M90, -90);
            case ROTATE_P90: return addRotation(graph, associatedNode, ROTATE_P90, 90);
            case ROTATE_180: return addRotation(graph, associatedNode, ROTATE_180, 180);
        }

        return SearchAction(action, associatedNode->action.position,
            associatedNode->action.g, associatedNode->action.h);
    }

    SearchAction(ActionEnum action = NONE,
            SearchPosition<GRAPH> position = SearchPosition<GRAPH>(),
            unsigned int g = 0, unsigned int h = 0) :
        actionType(action),
        position(position),
        g(g),
        h(h)
    {}

    unsigned int getTotalCost()
    {
        return Math::noOverflowAdd(g, h);
    }

    friend ostream& operator<<(ostream& stream, const SearchAction& searchAction)
    {
        return stream << searchAction.position << " " << ENUM_NAMES[searchAction.actionType];
    }

    ActionEnum actionType;
    SearchPosition<GRAPH> position;
    unsigned int g;
    unsigned int h;

private:

    static unordered_map<int, string> ENUM_NAMES;

    static SearchAction<GRAPH> addBackward(GRAPH* graph, SearchNode<GRAPH>* associatedNode)
    {
        SearchAction<GRAPH> action(BACKWARD);
        int currentOrientation = associatedNode->action.position.orientation;

        int direction = Math::normalizeAngleDeg(currentOrientation - 180);

        typename GRAPH::LocationType neighbor;
        if (graph->getNeighbor(associatedNode->action.position.location, direction, neighbor))
        {
            SearchPosition<GRAPH> newPosition(neighbor, currentOrientation);

            const SearchPositionNote* positionNote = reinterpret_cast<const SearchPositionNote*>(
                graph->getNote(neighbor));

            unsigned int locationCost = graph->getCost(neighbor);
            locationCost = getLocationCost(locationCost, positionNote);

            unsigned int commandCost = getCommandCost(BACKWARD, positionNote);
            unsigned int totalCost = Math::noOverflowAdd(locationCost, commandCost);

            action.position = newPosition;
            action.g = Math::noOverflowAdd(associatedNode->action.g, totalCost);
            action.h = associatedNode->action.h;

            return action;
        }

        return SearchAction<GRAPH>(NONE);
    }

    static SearchAction<GRAPH> addForward(GRAPH* graph, SearchNode<GRAPH>* associatedNode)
    {
        SearchAction<GRAPH> action(FORWARD);
        int currentOrientation = associatedNode->action.position.orientation;

        typename GRAPH::LocationType neighbor;
        if (graph->getNeighbor(associatedNode->action.position.location, currentOrientation, neighbor))
        {
            SearchPosition<GRAPH> newPosition(neighbor, currentOrientation);

            const SearchPositionNote* positionNote = reinterpret_cast<const SearchPositionNote*>(
                graph->getNote(neighbor));

            unsigned int locationCost = graph->getCost(neighbor);
            locationCost = getLocationCost(locationCost, positionNote);

            unsigned int commandCost = getCommandCost(FORWARD, positionNote);
            unsigned int totalCost = Math::noOverflowAdd(locationCost, commandCost);

            action.position = newPosition;
            action.g = Math::noOverflowAdd(associatedNode->action.g, totalCost);
            action.h = associatedNode->action.h;

            return action;
        }

        return SearchAction<GRAPH>(NONE);
    }

    static SearchAction<GRAPH> addRotation(GRAPH* graph, SearchNode<GRAPH>* associatedNode,
        ActionEnum actionEnum, int angle)
    {
        SearchAction<GRAPH> action(actionEnum);
        int currentOrientation = associatedNode->action.position.orientation;

        int newOrientation = Math::normalizeAngleDeg(currentOrientation + angle);
        SearchPosition<GRAPH> newPosition(associatedNode->action.position, newOrientation);

        const SearchPositionNote* positionNote = reinterpret_cast<const SearchPositionNote*>(
            graph->getNote(associatedNode->action.position.location));

        unsigned int commandCost = getCommandCost(actionEnum, positionNote);

        action.position = newPosition;
        action.g = Math::noOverflowAdd(associatedNode->action.g, commandCost);
        action.h = associatedNode->action.h;

        return action;
    }

    static unsigned int getCommandCost(ActionEnum action,
        const SearchPositionNote* positionNote)
    {
        unsigned int forward = 1;
        unsigned int backward = 40;
        unsigned int rotateM90 = 4;
        unsigned int rotateP90 = 4;
        unsigned int rotate180 = 4;

        if (positionNote)
        {
            if (positionNote->noRotations())
            {
                rotateM90 = MAX_COST;
                rotateP90 = MAX_COST;
                rotate180 = MAX_COST;
            }
        }

        switch (action)
        {
            case FORWARD: return forward;
            case BACKWARD: return backward;
            case ROTATE_M90: return rotateM90;
            case ROTATE_P90: return rotateP90;
            case ROTATE_180: return rotate180;
        }
    }

    static unsigned int getLocationCost(unsigned int locationCost,
        const SearchPositionNote* positionNote)
    {
        unsigned int staticObstacle = 0;

        if (positionNote)
        {
            staticObstacle = positionNote->staticObstacle() ? MAX_COST : 0;
        }

        return Math::noOverflowAdd(locationCost, staticObstacle);
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
    {SearchAction<GRAPH>::START, "START"},
    {SearchAction<GRAPH>::GOAL, "GOAL"},
    {SearchAction<GRAPH>::FORWARD, "FORWARD"},
    {SearchAction<GRAPH>::BACKWARD, "BACKWARD"},
    {SearchAction<GRAPH>::ROTATE_M90, "ROTATE_M90"},
    {SearchAction<GRAPH>::ROTATE_P90, "ROTATE_P90"},
    {SearchAction<GRAPH>::ROTATE_180, "ROTATE_180"}
};

} // namespace srs

#endif // SEARCHACTION_HPP_
