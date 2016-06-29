/*
 * (c) Copyright 2015-2016 River Systems, all rights reserved.
 *
 * This is proprietary software, unauthorized distribution is not permitted.
 */
#ifndef MAPNOTE_HPP_
#define MAPNOTE_HPP_

#include <string>
#include <sstream>
using namespace std;

namespace srs {

class MapNote
{
public:
    static const MapNote DISABLE_OD;
    static const MapNote ENABLE_OD;
    static const MapNote GO_SLOW;
    static const MapNote NO_ROTATIONS;
    static const MapNote STATIC_OBSTACLE;
    static const MapNote PREFERRED_0;
    static const MapNote PREFERRED_90;
    static const MapNote PREFERRED_180;
    static const MapNote PREFERRED_270;

    static MapNote* instanceOf(int8_t flags);

    MapNote() :
        od_(false),
        goSlow_(false),
        noRotations_(false),
        staticObstacle_(false),
        preferred_(false),
        preferredAngle_(0)
    {}

    MapNote(bool od,
            bool goSlow,
            bool noRotations,
            bool staticObstacle,
            bool preferred,
            int preferredAngle = 0) :
        od_(od),
        goSlow_(goSlow),
        noRotations_(noRotations),
        staticObstacle_(staticObstacle),
        preferred_(preferred),
        preferredAngle_(preferredAngle)
    {}

    void add(int8_t flags);

    int8_t getFlags();

    inline bool goSlow() const
    {
        return goSlow_;
    }

    void join(const MapNote& note)
    {
        goSlow_ |= note.goSlow_;
        noRotations_ |= note.noRotations_;
        od_ |= note.od_;
        staticObstacle_ |= note.staticObstacle_;
        preferred_ |= note.preferred_;
        preferredAngle_ = note.preferredAngle_;
    }

    inline bool od() const
    {
        return od_;
    }

    inline bool noRotations() const
    {
        return noRotations_;
    }

    inline bool preferred() const
    {
        return preferred_;
    }

    inline int preferredAngle() const
    {
        return preferredAngle_;
    }

    inline void reset()
    {
        goSlow_ = false;
        noRotations_ = false;
        od_ = false;
        staticObstacle_ = false;
        preferred_ = false;
        preferredAngle_ = 0;
    }

    inline bool staticObstacle() const
    {
        return staticObstacle_;
    }

    friend ostream& operator<<(ostream& stream, const MapNote& mapNote)
    {
        stream << "{" <<
            (mapNote.goSlow_ ? "GS " : "   ") <<
            (mapNote.noRotations_ ? "NR " : "   ") <<
            (mapNote.od_ ? "OD " : "   ") <<
            (mapNote.staticObstacle_ ? "SO" : "  ");

        if (mapNote.preferred_)
        {
            stream << "P";
            stream << mapNote.preferredAngle_ << " ";
            stream << " ";
        }

        return stream << "}";
    }

private:
    static constexpr unsigned char FLAG_OD = 0x80;
    static constexpr unsigned char FLAG_GO_SLOW = 0x40;
    static constexpr unsigned char FLAG_NO_ROTATIONS = 0x20;

    // TODO Define a proper map format
    static constexpr unsigned char FLAG_PREFERRED_2 = 0x10;
    static constexpr unsigned char FLAG_PREFERRED_1 = 0x08;
    static constexpr unsigned char FLAG_PREFERRED_0 = 0x04;

    static constexpr unsigned char FLAG_PREFERRED_000 = 0x00;
    static constexpr unsigned char FLAG_PREFERRED_090 = 0x04;
    static constexpr unsigned char FLAG_PREFERRED_180 = 0x08;
    static constexpr unsigned char FLAG_PREFERRED_270 = 0x0C;

    static constexpr unsigned char FLAG_STATIC_OBSTACLE = 0x01;

    static int getAngle(int8_t flags)
    {
        switch (MapNote::getAngleCode(flags))
        {
            case 0: return 0;
            case 1: return 90;
            case 2: return 180;
            case 3: return 270;
        }

        // TODO: Should throw an exception
        return -1;
    }

    static unsigned char getAngleCode(int8_t flags)
    {
        return (flags & (FLAG_PREFERRED_1 | FLAG_PREFERRED_0)) >> 2;
    }

    bool goSlow_;
    bool noRotations_;
    bool od_;
    bool staticObstacle_;
    bool preferred_;
    unsigned int preferredAngle_;
};

} // namespace srs

#endif // MAPNOTE_HPP_
