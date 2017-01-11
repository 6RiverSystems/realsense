/*
 * (c) Copyright 2015-2016 River Systems, all rights reserved.
 *
 * This is proprietary software, unauthorized distribution is not permitted.
 */
#pragma once

#include <string>
#include <sstream>
#include <unordered_map>
#include <memory>
using namespace std;

#include <srslib_framework/datastructure/graph/grid2d/WeightedGrid2d.hpp>
#include <srslib_framework/datastructure/Location.hpp>
#include <srslib_framework/datastructure/Position.hpp>
#include <srslib_framework/localization/map/BaseMap.hpp>
#include <srslib_framework/localization/map/mapnote/MapNotes.hpp>
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
                "], n: " << *labeledArea.notes <<
                "}";
        }

        friend bool operator==(const LabeledArea& lhs, const LabeledArea& rhs)
        {
            return lhs.label == rhs.label &&
                lhs.notes == rhs.notes &&
                lhs.ci == rhs.ci &&
                lhs.ri == rhs.ri &&
                lhs.cf == rhs.cf &&
                lhs.rf== rhs.rf;
        }

        string label;

        shared_ptr<MapNotes> notes;

        unsigned int cf;
        unsigned int ci;

        unsigned int rf;
        unsigned int ri;
    };

    using LabeledAreaMapType = unordered_map<string, LabeledArea>;

    LogicalMap(LogicalMetadata metadata, WeightedGrid2d* grid);
    ~LogicalMap()
    {}

    void addLabeledArea(unsigned int ciCells, unsigned int riCells,
        unsigned int cfCells, unsigned int rfCells,
        string label, shared_ptr<MapNotes> notes);

    void checkAreas(unsigned int cCells, unsigned int rCells, LabeledAreaMapType& areas) const;
    void checkAreas(double xM, double yM, LabeledAreaMapType& areas) const;
    void costSet(unsigned int cCells, unsigned int rCells, WeightedGrid2d::BaseType cost);
    void costMax(unsigned int cCells, unsigned int rCells, WeightedGrid2d::BaseType cost);

    LabeledAreaMapType getAreas() const
    {
        return labeledAreas_;
    }

    bool getNeighbor(const Position& position, Position& result);
    WeightedGrid2d::BaseType getCost(const Position& position) const;
    WeightedGrid2d::BaseType getCost(unsigned int cCells, unsigned int rCells) const;

    inline WeightedGrid2d* getGrid() const
    {
        return static_cast<WeightedGrid2d*>(grid_);
    }

    LogicalMetadata getMetadata() const
    {
        return metadata_;
    }

    WeightedGrid2d::BaseType getWeight(const Position& position) const;
    void getWeights(unsigned int cCells, unsigned int rCells,
        WeightedGrid2d::BaseType& north, WeightedGrid2d::BaseType& east,
        WeightedGrid2d::BaseType& south, WeightedGrid2d::BaseType& west);
    WeightedGrid2d::BaseType getWeights(unsigned int cCells, unsigned int rCells,
        int orientation) const;

    friend ostream& operator<<(ostream& stream, const LogicalMap& map);
    friend bool operator==(const LogicalMap& lhs, const LogicalMap& rhs);

    void setWeights(unsigned int cCells, unsigned int rCells,
        WeightedGrid2d::BaseType north,
        WeightedGrid2d::BaseType east,
        WeightedGrid2d::BaseType south,
        WeightedGrid2d::BaseType west);

private:
    LabeledAreaMapType labeledAreas_;

    LogicalMetadata metadata_;
};

ostream& operator<<(ostream& stream, const LogicalMap& map);

} // namespace srs
