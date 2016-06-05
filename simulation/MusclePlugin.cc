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

#include "MusclePlugin.hh"


using namespace gazebo;

//Register plugin with this simulator
GZ_REGISTER_MODEL_PLUGIN(MusclePlugin);

MusclePlugin::MusclePlugin()
{
}
// ITendon tendon = new ITendon();

MusclePlugin::state_type x(2);



void MusclePlugin::Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf)
{
	
	//Store pointer to the model
	this->model = _parent;

	//Get the parameters from SDF
	gzmsg << "Reading values from SDF" << std::endl;

	if (_sdf->HasElement("motor"))
	{	
		sdf::ElementPtr motorElement = _sdf->GetElement("motor");

		if (//!motorElement->HasElement("electric_current") || 
			!motorElement->HasElement("torque_constant") ||
			!motorElement->HasElement("bemf_constant") ||
			!motorElement->HasElement("inductance") ||
			!motorElement->HasElement("resistance") ||
			!motorElement->HasElement("inertiaMoment"))

		{
			gzwarn << "Invalid SDF: Missing required elements for motor model";
		}

		if (motorElement->HasElement("torque_constant"))
		{	
			_motor.torqueConst = motorElement->Get<float>("torque_constant");	
			gzdbg << "torque_constant " <<_motor.torqueConst<< "\n";
		}

		if (motorElement->HasElement("bemf_constant"))
		{
			_motor.BEMFConst = motorElement->Get<float>("bemf_constant");	
			gzdbg << "bemf_constant " <<_motor.BEMFConst<< "\n";
		}

		if (motorElement->HasElement("inductance"))
		{	
			_motor.inductance = motorElement->Get<float>("inductance");	
			gzdbg << "inductance " <<_motor.inductance<< "\n";
		}

		if (motorElement->HasElement("resistance"))
		{	
			_motor.resistance = motorElement->Get<float>("resistance");	
			gzdbg << "resistance " <<_motor.resistance<< "\n";
		}

		if (motorElement->HasElement("inertiaMoment"))
		{	
			_motor.inertiaMoment = motorElement->Get<float>("inertiaMoment");	
			gzdbg << "inertia " <<_motor.inertiaMoment<< "\n";
		}
	}

	if (_sdf->HasElement("gear"))
	{	
		sdf::ElementPtr gearElement = _sdf->GetElement("gear");

		if (!gearElement->HasElement("ratio") ||
			!gearElement->HasElement("efficiency") ||
			!gearElement->HasElement("inertiaMoment"))

		{
			gzwarn << "Invalid SDF: Missing required elements for gear model";
		}
		
		if (gearElement->HasElement("ratio"))
		{	
			_gear.ratio = gearElement->Get<float>("ratio");	
			gzdbg << "ratio " <<_gear.ratio<< "\n";
		}

		if (gearElement->HasElement("efficiency"))
		{
			_gear.efficiency = gearElement->Get<float>("efficiency");	
			gzdbg << "efficiency " <<_gear.efficiency<< "\n";
		}

		if (gearElement->HasElement("inertiaMoment"))
		{	
			_gear.inertiaMoment = gearElement->Get<float>("inertiaMoment");	
			gzdbg << "inertia " <<_gear.inertiaMoment<< "\n";
		}
	}


	if (_sdf->HasElement("spindle"))
	{	
		sdf::ElementPtr spindleElement = _sdf->GetElement("spindle");

		if (!spindleElement->HasElement("radius"))

		{
			gzwarn << "Invalid SDF: Missing required elements for spindle model";
		}

		if (spindleElement->HasElement("radius"))
		{	
			_spindle.radius = spindleElement->Get<float>("radius");	
			gzdbg << "radius " <<_spindle.radius<< "\n";
		}
	}

	if (_sdf->HasElement("SEE"))
	{	
		
		sdf::ElementPtr elasticElement = _sdf->GetElement("SEE");

        
		if (!elasticElement->HasElement("stiffness") ||
			 !elasticElement->HasElement("length0"))
		{
			gzwarn << "Invalid SDF: Missing required elements for series elastic element";
		}

            //stiffness
			// sdf::ElementPtr stiffnessElement = elasticElement->GetElement("stiffness");
			//float stiffness[4];

		if (elasticElement->HasElement("stiffness"))
		{
			_see.stiffness = elasticElement->Get<float>("stiffness");
		}
		

			//get floats from string
			// std::istringstream ss(stiffnessString);
			// std::copy(std::istream_iterator <float> (ss),
			// 	std::istream_iterator <float>(),
			// 	SEE.stiffness);

		if (elasticElement->HasElement("length0"))
		{	
			_see.lengthRest = elasticElement->Get<float>("length0");
		}

	}

	// read from TendonInfo and insert elements to linkRelvec, and visualMsg
	std::ifstream myfile ("TendonInfo.txt");
	if (myfile.is_open())
  	{

  		int i = 0;
  		int j = 0;
  		for (std::string line; getline (myfile,line);)
  		{
  			vector<math::Vector3> emptyVector;
  			vector<std::string> emptyString;
  			vector<physics::LinkPtr> emptyLinks;
  			vector<msgs::Visual> emptyVisual;
  			linkRelVec.push_back(emptyVector);
			visualMsg.push_back(emptyVisual);

			std::stringstream linestringstream(line);

	    	for ( std::string element; getline (linestringstream,element,' ');)
	    	{
	      		char ch;
	      		double x,y,z;
	      		std::vector<int> array;
	      		std::stringstream ss(element);
	      		math::Vector3 relVecTmp;

	    		if (ss>>x>>ch>>y>>ch>>z)
	    		{
	    			relVecTmp.Set(x,y,z);
	    			//If input to string successfully then add coordinate to "linkRelVec"
	    			linkRelVec.back().push_back(relVecTmp);
	    		}else
	    		{
	    			physics::LinkPtr link = this->model->GetLink(element);
			    	if (!link)
			    	{
				        gzwarn << "Invalid SDF: model link " << element << " does not "
				               << "exist!" << std::endl;
				        continue;
			    	}

			    	// if link not Null add it to "this->links"
			    	this->links.push_back(link);

			    	// create a visual mesage object "visualMsgTmp" and configure
			    	std::stringstream ss;
					ss << "CYLINDER_VISUAL__" << i<< j;
					std::string visualName = ss.str();
					msgs::Visual visualMsgTmp;
			    	visualMsgTmp.set_name(visualName);
			    	//Set the visual's parent. This visual will be attached to the parent
			    	//visualMsg.set_parent_name(_parent->GetScopedName());
			    	visualMsgTmp.set_parent_name(link->GetName());
			    	msgs::Geometry *geomMsg = visualMsgTmp.mutable_geometry();
			    	// Create a cylinder
			    	geomMsg->set_type(msgs::Geometry::CYLINDER);
			    	geomMsg->mutable_cylinder()->set_radius(.001);
			    	visualMsgTmp.set_cast_shadows(false);
			    	// then add it to "visualMsg"
			    	visualMsg.back().push_back(visualMsgTmp);
			    	j++;
	    		}
	    		i++;
	    	}
	    }
    	std::cout << "Link relative vectors: \n" 
    		<< linkRelVec[0][0] << "\t" 
    		<< linkRelVec[0][1] << "\t" 
    		<< linkRelVec[0][2] << "\n"
    		<< linkRelVec[1][0] << "\t" 
    		<< linkRelVec[1][1] << "\t" 
    		<< linkRelVec[1][2] << "\n";

    	myfile.close();
  	}
  	else std::cout << "Unable to open file";

	this->connection = event::Events::ConnectWorldUpdateBegin(
         			 boost::bind(&MusclePlugin::OnUpdate, this));

	//create a cylinder as visual

    node = transport::NodePtr(new transport::Node());
    node->Init(_parent->GetWorld()->GetName());
    visPub = node->Advertise<msgs::Visual>("~/visual", 10);

    // Set the visual's name. This should be unique.
 //    for (int i=0; i < linkNumber - 1; i++)
 //    {
 //    	std::stringstream ss;
	// 	ss << "CYLINDER_VISUAL__" << i;
	// 	std::string visualName = ss.str();
 //    	visualMsg[i].set_name(visualName);

 //    	//Set the visual's parent. This visual will be attached to the parent
 //    	//visualMsg.set_parent_name(_parent->GetScopedName());
 //    	visualMsg[i].set_parent_name(links[i]->GetName());
 //    	msgs::Geometry *geomMsg = visualMsg[i].mutable_geometry();

 //    	//std::cout << _parent->GetScopedName();
 //    	// Create a cylinder

 //    	geomMsg->set_type(msgs::Geometry::CYLINDER);
 //    	geomMsg->mutable_cylinder()->set_radius(.001);
 //    	visualMsg[i].set_cast_shadows(false);
	// }
}

void MusclePlugin::Init()
{
	//state initialization
    x[0] = 0.0; // start at i=0.0, w_g=0.0
    x[1] = 0.0;
    _motor.voltage = 0.0;
    _spindle.angVel = 0;

    // get bounding box of the link 
    // gzmsg << "CoG 1 : " << links[0]->GetWorldCoGPose() << "\n";
    // gzmsg << "CoG 2 : " << links[1]->GetWorldCoGPose() << "\n";
    // rendering::WireBox::WireBox	(this->model, links[0]-> GetBoundingBox());
	// links[0]-> ShowBoundingBox();
	// links[1]-> ShowBoundingBox();

	// insertion = math::Vector3(1, 1, 1);//armPose.pos;
	// fixation = math::Vector3(2, 2, 2);//handPose.pos;
	
	

    
}


void MusclePlugin::OnUpdate()
{
	
	
	//compute the step time

	common::Time currTime = this->model->GetWorld()->GetSimTime();
    common::Time stepTime = currTime - this->prevUpdateTime;
 	this->prevUpdateTime = currTime;
	
	if ( fmod(currTime.Double(),0.2) == 0 )
    {
		std::cout << "Voltage:";
		std::cin >> _motor.voltage;
	}

	math::Vector3 force;
	vector<vector<math::Vector3>> viaPointPos;
	vector<tendonType> newTendon;

	int linkIndex = 0;
	for (int i = 0; i < linkRelVec.size(); i++)
	{
    	vector<math::Vector3> emptyViaPointPos;
    	viaPointPos.push_back(emptyViaPointPos);
    	for (int j = 0; j < linkRelVec[i].size(); j++) 
    	{
        	const math::Pose linkPose= this->links[linkIndex]->GetWorldCoGPose();
        	cout << "this is the relative coordinate:  " << linkPose << "\n";
        	viaPointPos.back().push_back(linkPose.pos+linkPose.rot.RotateVector(linkRelVec[i][j]));
    		linkIndex++;
    	}
	}
	linkIndex = 0;

	GetTendonInfo(viaPointPos, &newTendon);	

	// extract info from linkPose. not in use for now!
	_see.length = newTendon[0].Vector[0].GetLength() + newTendon[0].Vector[1].GetLength();


	// calculate elastic force 
	elasticForce = 0;//tendon.ElasticElementModel(_see.lengthRest, 
		// _see.length, _see.stiffness, _spindle.angVel,
		// _spindle.radius, stepTime.Double());
	
	
	// calculate motor force
	// calculate the approximation of gear's efficiency
	_gear.appEfficiency = EfficiencyApproximation(); 

	// do 1 step of integration of DiffModel() at current time
	stepper.do_step(DiffModel, x, currTime.Double(), 
		stepTime.Double());
	
	 // gzdbg << "electric current: " 
		// 	  << x[0]
		// 	  << "\t"
		// 	  << "speed: "
		// 	  << x[1]
		// 	  << "\n";

	_motor.current = x[0];
 	_spindle.angVel = x[1];

 	actuatorForce = ElectricMotorModel(_motor.current, _motor.torqueConst,
 		_spindle.radius);
    cout << "orientation" << "\n"
    	<< newTendon[0].Orientation[0] << "\n"
    	<< newTendon[0].Orientation[1] << "\n"
    	<< newTendon[0].Orientation[2] << "\n"
    	<< newTendon[1].Orientation[0] << "\n"
    	<< newTendon[1].Orientation[1] << "\n"
    	<< newTendon[1].Orientation[2] << "\n"
    	<< "actuator force" << "\n"
    	<< actuatorForce << "\n";
    // calculate general force (elastic+actuator)
    for (int i = 0; i < visualMsg.size(); i++)
    {
	 	for (int j = 0; j < 2; j++)
	 	{
	 		cout << linkIndex<<"\n";
	 		//remove elastic force, this is to be discussed
			force = CalculateForce(elasticForce, actuatorForce, newTendon[i].Orientation[j]);

		    this->links[linkIndex]->AddForceAtWorldPosition(-force, viaPointPos[i][j]);
		    this->links[linkIndex+1]->AddForceAtWorldPosition(force,viaPointPos[i][j+1]);

		    cout << "this is the real force:  " << force << "\n"
		    << "viaPointPos1:  " <<viaPointPos[i][j] << "\n"
		    << "viaPointPos2:  " <<viaPointPos[i][j+1] << "\n";
		    //update position and orientation, somehow float not accepted, must use double instead. for pricision maybe
		 //    msgs::Set(visualMsg[i][j].mutable_pose(),ignition::math::Pose3d(
		 //    		ignition::math::Vector3d(static_cast<double>(newTendon[i].MidPoint[j][0]),
		 //    			static_cast<double>(newTendon[i].MidPoint[j][1]),
		 //    			static_cast<double>(newTendon[i].MidPoint[j][2])),
		 //    		ignition::math::Vector3d(newTendon[i].Roll[j],newTendon[i].Pitch[j],0)));
		 //    msgs::Geometry *geomMsg = visualMsg[i][j].mutable_geometry();

		 //    //update length
			// geomMsg->mutable_cylinder()->set_length(newTendon[i].Vector[j].GetLength());
			// visPub->Publish(visualMsg[i][j]);//show updated visual i think "col is pointing to this object but somehow can not be published.."
			linkIndex++;
			
		}
		cout << linkIndex<<"\n";
		linkIndex++;
		
	}

}

void MusclePlugin::DiffModel( const state_type &x , state_type &dxdt , const double /* t */ )
{
    //x[0] - motor electric current
    //x[1] - spindle angular velocity
	float totalIM = _motor.inertiaMoment + _gear.inertiaMoment; // total moment of inertia
    dxdt[0] = 1/_motor.inductance * (-_motor.resistance * x[0] -_motor.BEMFConst * _gear.ratio * x[1] + _motor.voltage);
    dxdt[1] = _motor.torqueConst * x[0] / (_gear.ratio * totalIM) - 
    	_spindle.radius * elasticForce / (_gear.ratio * _gear.ratio * totalIM * _gear.appEfficiency);
}

float MusclePlugin::EfficiencyApproximation()
{
	float param1 = 0.1; // defines steepness of the approxiamtion
	float param2 = 0; // defines zero crossing of the approxiamtion
	return _gear.efficiency + (1/_gear.efficiency - _gear.efficiency)*(0.5*(tanh(-param1 * _spindle.angVel * _motor.current - param2) +1));
}