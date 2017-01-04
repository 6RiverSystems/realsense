/*
 * (c) Copyright 2015-2016 River Systems, all rights reserved.
 *
 * This is proprietary software, unauthorized distribution is not permitted.
 */
#pragma once

namespace srs {

template<class OWNER>
class SoftwareMessageHandler
{
public:
    SoftwareMessageHandler(OWNER* owner) :
        owner_(owner),
		valid_(false)
    {}

    virtual ~SoftwareMessageHandler()
    {}

    OWNER* getOwner() const
    {
        return owner_;
    }

    virtual void attach() {};

    virtual void sync()
    {
    	if (valid_)
    	{
    		syncState();
    	}
    };

protected:

    virtual void syncState() {};

    bool valid_;

private:

    OWNER* owner_;
};

} // namespace srs
