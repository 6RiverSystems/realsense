/*
 * (c) Copyright 2015-2016 River Systems, all rights reserved.
 *
 * This is proprietary software, unauthorized distribution is not permitted.
 */

#ifndef STARGAZER_POINT_TRANSFORMER_HPP_
#define STARGAZER_POINT_TRANSFORMER_HPP_

#include <ros/ros.h>
#include <StarGazerFilter.h>
#include <tf/tf.h>

namespace srs
{

class StarGazerPointTransformer
{

public:
	StarGazerPointTransformer( );

	virtual ~StarGazerPointTransformer( );

	void Load( const std::string& strTargetFrame, const tf::Transform& footprintTransform,
		const std::string& strAnchorsFile, const std::string& strCalibrationFile );

	double ConvertToRightHandRule( double fLeftHandAngleInDegrees ) const;

	tf::Quaternion ConvertAngle( double fLeftHandAngleInDegrees ) const;

	tf::Vector3 GetStargazerOffset( double fX, double fY, double fZ ) const;

	tf::Vector3 GetCameraOffset( const tf::Quaternion& angle ) const;

	tf::Vector3 GetFootprintOffset( const tf::Quaternion& angle ) const;

	bool TransformPoint( int tagID, double fX, double fY, double fZ, double fAngle, tf::Pose& pose );

	std::string GetTargetFrame( ) const;

	std::string GetAnchorFrame( int tagID ) const;

private:

	void LoadAnchors( const std::string& strAnchorsFile );

	void LoadCalibrationTransform( const std::string& strCalibrationFile );

private:

	StarGazerFilter					m_filter;

	std::string						m_strAnchorFrame;

	std::string						m_strTargetFrame;

	std::map<int, tf::Transform>	m_mapTransforms;

	tf::Transform					m_stargazerTransform;

	tf::Transform					m_footprintTransform;

	tf::Transform					m_rotationTransform;

};

} // namespace srs

#endif  // STARGAZER_POINT_TRANSFORMER_HPP_
