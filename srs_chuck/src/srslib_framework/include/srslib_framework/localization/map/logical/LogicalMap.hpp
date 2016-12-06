/*
 * (c) Copyright 2015-2016 River Systems, all rights reserved.
 *
 * This is proprietary software, unauthorized distribution is not permitted.
 */
#pragma once

#include <string>
#include <sstream>
#include <unordered_map>
using namespace std;

#include <srslib_framework/datastructure/graph/grid2d/Grid2d.hpp>
#include <srslib_framework/localization/map/BaseMap.hpp>
#include <srslib_framework/localization/map/MapNote.hpp>
#include <srslib_framework/localization/map/logical/LogicalMetadata.hpp>

namespace srs {

class LogicalMap : public BaseMap
{
public:
    struct LabeledArea
    {
        friend ostream& operator<<(ostream& stream, const LabeledArea& labeledArea)
        {
            return stream << "{" <<
                "l: '" << labeledArea.label <<
                "', a: [" << labeledArea.ci << ", " << labeledArea.ri << ", " <<
                    labeledArea.cf << ", " << labeledArea.rf <<
                "], n: " << labeledArea.note <<
                "}";
        }

        friend bool operator==(const LabeledArea& lhs, const LabeledArea& rhs)
        {
            return lhs.label == rhs.label &&
                lhs.note == rhs.note &&
                lhs.ci == rhs.ci &&
                lhs.ri == rhs.ri &&
                lhs.cf == rhs.cf &&
                lhs.rf== rhs.rf;
        }

        string label;

        MapNote note;

        unsigned int cf;
        unsigned int ci;

        unsigned int rf;
        unsigned int ri;
    };

    using LabeledAreaMapType = unordered_map<string, LabeledArea>;

    LogicalMap(double widthM, double heightM, double resolution, Pose<> origin);
    LogicalMap(Grid2d* grid, double resolution, Pose<> origin);
    LogicalMap(LogicalMetadata metadata);
    ~LogicalMap()
    {}

    void addLabeledArea(unsigned int ciCells, unsigned int riCells,
        unsigned int cfCells, unsigned int rfCells,
        string label, MapNote note);

    void checkAreas(unsigned int cCells, unsigned int rCells, LabeledAreaMapType& areas) const;
    void checkAreas(double xM, double yM, LabeledAreaMapType& areas) const;

    LabeledAreaMapType getAreas() const
    {
        return labeledAreas_;
    }

    Grid2d::BaseType getCost(unsigned int cCells, unsigned int rCells) const
    {
        return getGrid()->getPayload(Grid2d::Location(cCells, rCells));
    }

    LogicalMetadata getMetadata() const
    {
        return metadata_;
    }

    Grid2d::BaseType getWeights(unsigned int cCells, unsigned int rCells, int orientation) const
    {
        return getGrid()->getWeight(Grid2d::Position(cCells, rCells, orientation));
    }

    void maxCost(unsigned int cCells, unsigned int rCells, Grid2d::BaseType cost);

    friend ostream& operator<<(ostream& stream, const LogicalMap& map);
    friend bool operator==(const LogicalMap& lhs, const LogicalMap& rhs);

    void setCost(unsigned int cCells, unsigned int rCells, Grid2d::BaseType cost);
    void setObstacle(unsigned int cCells, unsigned int rCells);
    void setWeights(unsigned int cCells, unsigned int rCells,
        Grid2d::BaseType north,
        Grid2d::BaseType east,
        Grid2d::BaseType south,
        Grid2d::BaseType west);

private:
    LabeledAreaMapType labeledAreas_;

    LogicalMetadata metadata_;
};

ostream& operator<<(ostream& stream, const LogicalMap& map);

} // namespace srs
