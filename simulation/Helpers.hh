#include <sstream>
#include <ignition/math/Vector3.hh>
#include <gazebo/common/common.hh>
#include <boost/bind.hpp>
#include <math.h>
#include <map>
#include <stdio.h>
#include <algorithm> 



namespace gazebo
{
  
  using namespace std;
  using namespace boost::numeric::odeint; 
  
  

  struct Motor
    {
      float current;
      float torqueConst;
      float resistance;
      float inductance;
      float voltage;
      float BEMFConst; // back electromagnetic force constant
      float inertiaMoment;
    };

    struct Gear
    {
      float inertiaMoment;
      float ratio;
      float efficiency; // gear efficciency
      float appEfficiency; // approximated efficiency 
    };

    struct Spindle
    {
      float angVel; // angular velocity of the spindle
      float radius;
    };

    struct SEE
    {
	  float stiffness;
	  float length;
	  float lengthRest;
    };

    struct tendonType {
	  vector<math::Vector3> MidPoint;
	  vector<math::Vector3> Vector;//might need it to calculate length
	  vector<math::Vector3> Orientation;
	  vector<double> Pitch;
	  vector<double> Roll;
	  } ;

}