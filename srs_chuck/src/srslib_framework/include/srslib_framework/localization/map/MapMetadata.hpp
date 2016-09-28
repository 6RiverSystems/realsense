/*
 * (c) Copyright 2015-2016 River Systems, all rights reserved.
 *
 * This is proprietary software, unauthorized distribution is not permitted.
 */
#pragma once

#include <tf/tf.h>

namespace srs {

using namespace std;

struct MapMetadata
{
public:
    MapMetadata() :
        heightCells(0),
        widthCells(0),
        heightM(0.0),
        widthM(0.0),
        orientation(tf::createQuaternionFromYaw(0)),
        origin(tf::createQuaternionFromYaw(0)),
        mapDocumentFilename(""),
        mapImageFilename(""),
        resolution(0.0),
        thresholdOccupied(0.9),
        thresholdFree(0.1),
        negate(false)
    {}

    int heightCells;
    double heightM;

    string mapDocumentFilename;
    string mapImageFilename;

    bool negate;

    tf::Quaternion origin;
    tf::Quaternion orientation;

    double resolution;

    double thresholdFree;
    double thresholdOccupied;

    int widthCells;
    double widthM;
};

} // namespace srs
