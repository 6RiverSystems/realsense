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

#include <srslib_framework/math/BasicMath.hpp>
#include <srslib_framework/math/AngleMath.hpp>

#include <srslib_framework/search/SearchPosition.hpp>
#include <srslib_framework/localization/map/MapNote.hpp>

namespace srs {

template<typename GRAPH>
struct SearchNode;

template<typename GRAPH>
struct SearchAction
{
    constexpr static unsigned int COST_MAX = numeric_limits<unsigned int>::max();
    constexpr static unsigned int COST_FORWARD = 1;
    constexpr static unsigned int COST_FORWARD_NMA = 12; // Forward non-matching-angle
    constexpr static unsigned int COST_BACKWARD = 40;
    constexpr static unsigned int COST_ROTATE_N90 = 4;
    constexpr static unsigned int COST_ROTATE_P90 = 4;
    constexpr static unsigned int COST_ROTATE_180 = 3;

    enum ActionEnum {
        BACKWARD,
        FORWARD,
        GOAL,
        NONE,
        ROTATE_N90, ROTATE_P90, ROTATE_180,
        START
    };

    typedef typename GRAPH::Location LocationType;

    static array<ActionEnum, 5> ALLOWED_ACTIONS;

    static SearchAction<GRAPH>* instanceOf(ActionEnum action,
        GRAPH* graph, SearchNode<GRAPH>* parentNode)
    {
        switch (action)
        {
            case FORWARD: return addForward(graph, parentNode);
            case BACKWARD: return addBackward(graph, parentNode);
            case ROTATE_N90: return addRotation(graph, parentNode, ROTATE_N90, -90);
            case ROTATE_P90: return addRotation(graph, parentNode, ROTATE_P90, 90);
            case ROTATE_180: return addRotation(graph, parentNode, ROTATE_180, 180);
        }

        return new SearchAction(action,
            parentNode->action->position,
            parentNode->action->g, parentNode->action->h);
    }

    static SearchAction<GRAPH>* instanceOf(ActionEnum action,
        GRAPH* graph,
        SearchPosition<GRAPH> position = SearchPosition<GRAPH>(),
        unsigned int h = 0)
    {
        unsigned int g = getTotalCost(graph, position, ActionEnum::NONE);
        return new SearchAction(action, position, g, h);
    }

    SearchAction(ActionEnum action, SearchPosition<GRAPH> position = SearchPosition<GRAPH>(),
            unsigned int g = 0, unsigned int h = 0) :
        actionType(action),
        position(position),
        g(g),
        h(h)
    {}

    unsigned int getTotalCost()
    {
        return BasicMath::noOverflowAdd<int>(g, h);
    }

    friend ostream& operator<<(ostream& stream, const SearchAction* searchAction)
    {
        return stream << searchAction->position << " " <<
            ENUM_NAMES[searchAction->actionType] <<
            " (g: " << searchAction->g << ", h: " << searchAction->h << ") ";
    }

    ActionEnum actionType;
    SearchPosition<GRAPH> position;
    unsigned int g;
    unsigned int h;

private:

    static unordered_map<int, string> ENUM_NAMES;

    static SearchAction<GRAPH>* addBackward(GRAPH* graph, SearchNode<GRAPH>* parentNode)
    {
        int currentOrientation = parentNode->action->position.orientation;
        int direction = AngleMath::normalizeDeg(currentOrientation - 180);

        LocationType neighbor;
        if (graph->getNeighbor(parentNode->action->position.location, direction, neighbor))
        {
            SearchAction<GRAPH>* newAction = new SearchAction<GRAPH>(BACKWARD);
            SearchPosition<GRAPH> newPosition(neighbor, currentOrientation);

            unsigned int totalCost = getTotalCost(graph, newPosition, BACKWARD);

            newAction->position = newPosition;
            newAction->g = BasicMath::noOverflowAdd<int>(parentNode->action->g, totalCost);
            newAction->h = parentNode->action->h;

            return newAction;
        }

        return nullptr;
    }

    static SearchAction<GRAPH>* addForward(GRAPH* graph, SearchNode<GRAPH>* parentNode)
    {
        int currentOrientation = parentNode->action->position.orientation;

        LocationType neighbor;
        if (graph->getNeighbor(parentNode->action->position.location, currentOrientation, neighbor))
        {
            SearchAction<GRAPH>* newAction = new SearchAction<GRAPH>(FORWARD);
            SearchPosition<GRAPH> newPosition(neighbor, currentOrientation);

            unsigned int totalCost = getTotalCost(graph, newPosition, FORWARD);

            newAction->position = newPosition;
            newAction->g = BasicMath::noOverflowAdd<int>(parentNode->action->g, totalCost);
            newAction->h = parentNode->action->h;

            return newAction;
        }

        return nullptr;
    }

    static SearchAction<GRAPH>* addRotation(GRAPH* graph, SearchNode<GRAPH>* parentNode,
        ActionEnum actionEnum, int angle)
    {
        SearchAction<GRAPH>* newAction = new SearchAction<GRAPH>(actionEnum);
        int currentOrientation = parentNode->action->position.orientation;

        int newOrientation = AngleMath::normalizeDeg(currentOrientation + angle);
        SearchPosition<GRAPH> newPosition(parentNode->action->position, newOrientation);

        // WARNING: If multiple rotations are performed over the same location,
        // the total cost will add the location cost every time. For now, this
        // behavior is fine
        unsigned int totalCost = getTotalCost(graph, newPosition, actionEnum);

        newAction->position = newPosition;
        newAction->g = BasicMath::noOverflowAdd<int>(parentNode->action->g, totalCost);
        newAction->h = parentNode->action->h;

        return newAction;
    }

    static unsigned int getTotalCost(GRAPH* graph, SearchPosition<GRAPH> position, ActionEnum action)
    {
        unsigned int locationCost = graph->getCost(position.location);

        // ###FS Fix the MapNote issue
        // const MapNote* note = reinterpret_cast<const MapNote*>(graph->getNote(position.location));

        unsigned int actionCost = 0;
        switch (action)
        {
            case FORWARD:
            {
                actionCost = COST_FORWARD;
// ###FS
//                if (note && note->preferred())
//                {
//                    if (position.orientation != note->preferredAngle())
//                    {
//                        actionCost = COST_FORWARD_NMA;
//                    }
//                }
                break;
            }

            case BACKWARD:
            {
                actionCost = COST_BACKWARD;
                break;
            }

            case ROTATE_N90:
            {
                actionCost = COST_ROTATE_N90;
// ###FS
//                if (note && note->noRotations())
//                {
//                    actionCost = COST_MAX;
//                }
                break;
            }

            case ROTATE_P90:
            {
                actionCost = COST_ROTATE_P90;
// ###FS
//                if (note && note->noRotations())
//                {
//                    actionCost = COST_MAX;
//                }
                break;
            }

            case ROTATE_180:
            {
                actionCost = COST_ROTATE_180;
// ###FS
//                if (note && note->noRotations())
//                {
//                    actionCost = COST_MAX;
//                }
                break;
            }
        }

        return BasicMath::noOverflowAdd<int>(locationCost, actionCost);
    }
};

template<typename GRAPH>
array<typename SearchAction<GRAPH>::ActionEnum, 5> SearchAction<GRAPH>::ALLOWED_ACTIONS = {
    FORWARD,
    ROTATE_N90,
    ROTATE_P90,
    ROTATE_180
};

template<typename GRAPH>
unordered_map<int, string> SearchAction<GRAPH>::ENUM_NAMES = {
    {SearchAction<GRAPH>::BACKWARD, "BACKWARD"},
    {SearchAction<GRAPH>::FORWARD, "FORWARD"},
    {SearchAction<GRAPH>::GOAL, "GOAL"},
    {SearchAction<GRAPH>::NONE, "NONE"},
    {SearchAction<GRAPH>::ROTATE_M90, "ROTATE_N90"},
    {SearchAction<GRAPH>::ROTATE_P90, "ROTATE_P90"},
    {SearchAction<GRAPH>::ROTATE_180, "ROTATE_180"},
    {SearchAction<GRAPH>::START, "START"}
};

} // namespace srs

#endif // SEARCHACTION_HPP_
