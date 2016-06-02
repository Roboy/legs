#include <sstream>
#include <gazebo/gazebo.hh>
#include <gazebo/common/common.hh>
#include <gazebo/physics/physics.hh>
#include <ignition/math/Vector3.hh>
#include <boost/bind.hpp>
#include <math.h>
#include <stdio.h>
#include <algorithm> 
#include <boost/numeric/odeint.hpp>
#include <vector>
#include <iostream>
#include <fstream>
#include <chrono>

#include "ITendon.hh"

using namespace gazebo;

ITendon::ITendon()
{
	
}

float ITendon::DotProduct(const math::Vector3 &_v1, const math::Vector3 &_v2)
{
	return _v1.x*_v2.x + _v1.y*_v2.y + _v1.z*_v2.z;
}


float ITendon::Angle(const math::Vector3 &_v1, const math::Vector3 &_v2)
{
	return acos(_v1.Dot(_v2)/_v1.GetLength()*_v2.GetLength());
}


float ITendon::ElectricMotorModel(const float _current,  const float _torqueConstant, 
					const float _spindleRadius)
{
    float motorForce;

    if (_current>=0)
    {
        motorForce=_current*_torqueConstant/_spindleRadius;
    }
    else
    {
        motorForce=0;
    }
    
	return motorForce;
}


float ITendon::ElasticElementModel(const float _length0, const float _length, float _stiffness,
							 const float _speed, const float _spindleRadius, const double _time)
{
    // float realTimeUpdateRate=1000;
    float windingLength = _spindleRadius*_speed*_time;
	float displacement;
	displacement = windingLength + _length - _length0;
    
	// gzdbg << "displacement: " 
	// 	  << displacement
	// 	  << "\n"
 //          << "windingLength: " 
	// 	  << windingLength
	// 	  << "\n";
    
    float elasticForce;

    if (displacement>=0)
    {
        elasticForce=displacement*_stiffness;
    }
    else
    {
        elasticForce=0;
    }

	//return _stiffness[0] + (displacement*_stiffness[1]) + (displacement*displacement*_stiffness[2]) + 
	//			(displacement*displacement*displacement*_stiffness[3]) ;
    //return displacement*_stiffness[0];
    return elasticForce;	 
                
}


math::Vector3 ITendon::CalculateForce(float _elasticForce, float _motorForce, 
	const math::Vector3 &_tendonOrien)
{
	// math::Vector3 diff = _fixationP - _instertionP;
    
	/*    float tendonForce;
    
    if (_elasticForce+_motorForce>=0)
    {
        tendonForce=_elasticForce+_motorForce;
    }
    else
    {
        tendonForce=0;
    }*/
    
	return _tendonOrien*(_elasticForce+_motorForce);
    
}

 void ITendon::GetTendonInfo(vector<vector<math::Vector3>> _viaPointPose, vector<tendonType> *_tendon_p)//try later with pointer
{
	for (int i = 0; i < _viaPointPose.size(); i++)
	{
		tendonType newTendon;
		_tendon_p->push_back(newTendon);
		for (int j = 0; j < _viaPointPose[i].size()-1; j++)
		{
				_tendon_p->back().MidPoint.push_back((_viaPointPose[i][j] + _viaPointPose[i][j+1])/2);
				_tendon_p->back().Vector.push_back(_viaPointPose[i][j] - _viaPointPose[i][j+1]);
				_tendon_p->back().Orientation.push_back(_tendon_p->back().Vector.back()/_tendon_p->back().Vector.back().GetLength());
				_tendon_p->back().Pitch.push_back(static_cast<double>(atan(_tendon_p->back().Orientation.back()[0]/_tendon_p->back().Orientation.back()[2])));
				_tendon_p->back().Roll.push_back(static_cast<double>(-acos(sqrt((pow(_tendon_p->back().Orientation.back()[0],2)+pow(_tendon_p->back().Orientation.back()[2],2))))));
		}
	}
}