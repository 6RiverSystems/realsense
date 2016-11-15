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
                "', a: [" << labeledArea.xi << ", " << labeledArea.yi << ", " <<
                    labeledArea.xf << ", " << labeledArea.yf <<
                "], n: " << labeledArea.note <<
                "}";
        }

        friend bool operator==(const LabeledArea& lhs, const LabeledArea& rhs)
        {
            return lhs.label == rhs.label &&
                lhs.note == rhs.note &&
                lhs.xi == rhs.xi &&
                lhs.yi == rhs.yi &&
                lhs.xf == rhs.xf &&
                lhs.yf== rhs.yf;
        }

        string label;

        MapNote note;

        unsigned int xf;
        unsigned int xi;

        unsigned int yf;
        unsigned int yi;
    };

    using LabeledAreaMapType = unordered_map<string, LabeledArea>;

    LogicalMap(double widthM, double heightM, double resolution);
    LogicalMap(Grid2d* grid, double resolution);
    LogicalMap(LogicalMetadata metadata);
    ~LogicalMap()
    {}

    void addLabeledArea(unsigned int xi, unsigned int yi, unsigned int xf, unsigned int yf,
        string label, MapNote note);

    void checkAreas(unsigned int c, unsigned int r, LabeledAreaMapType& areas) const;
    void checkAreas(double x, double y, LabeledAreaMapType& areas) const;

    LabeledAreaMapType getAreas() const
    {
        return labeledAreas_;
    }

    Grid2d::BaseType getCost(unsigned int c, unsigned int r) const
    {
        return getGrid()->getPayload(Grid2d::Location(c, r));
    }

    LogicalMetadata getMetadata() const
    {
        return metadata_;
    }

    Grid2d::BaseType getWeights(unsigned int c, unsigned int r, int orientation) const
    {
        return getGrid()->getWeight(Grid2d::Position(c, r, orientation));
    }

    void maxCost(unsigned int c, unsigned int r, Grid2d::BaseType cost);

    friend ostream& operator<<(ostream& stream, const LogicalMap& map);
    friend bool operator==(const LogicalMap& lhs, const LogicalMap& rhs);

    void setCost(unsigned int c, unsigned int r, Grid2d::BaseType cost);
    void setObstruction(unsigned int c, unsigned int r);
    void setWeights(unsigned int c, unsigned int r,
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
