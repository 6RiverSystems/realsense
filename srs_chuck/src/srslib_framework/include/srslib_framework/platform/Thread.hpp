/*
 * (c) Copyright 2015-2016 River Systems, all rights reserved.
 *
 * This is proprietary software, unauthorized distribution is not permitted.
 */
#ifndef THREAD_HPP_
#define THREAD_HPP_

#include <functional>
#include <ros/ros.h>
#include <ros/callback_queue.h>
#include <ros/callback_queue_interface.h>

namespace srs {

	class GenericCallback : public ros::CallbackInterface
	{
	private:

		std::function<void()> m_callback;

	public:

		explicit GenericCallback( const std::function<void()>& callback ) :
			m_callback( callback ) { }

		virtual ~GenericCallback( ) { }

		virtual CallResult call( )
		{
			if( m_callback )
			{
				m_callback( );

				return Success;
			}
			else
			{
				return Invalid;
			}
		}
	};

	inline void ExecuteInRosThread( std::function<void ()> function )
	{
		ros::CallbackQueue* pCallbackQueue = ros::getGlobalCallbackQueue( );

		if( pCallbackQueue )
		{
			pCallbackQueue->addCallback( ros::CallbackInterfacePtr( new GenericCallback(
				function ) ) );
		}
		else
		{
			ROS_ERROR( "Error accessing the callback queue: ros::getGlobalCallbackQueue" );
		}
	}

} // namespace srs

#endif // THREAD_HPP_
