/*
** ASVDynamicsPlugin.cc
** Login : <daneshtarapore@gmail.com>
** Started on  Nov 03 2016 Danesh Tarapore
**
** Copyright (C) 2016 Danesh Tarapore
** This program is free software; you can redistribute it and/or modify
** it under the terms of the GNU General Public License as published by
** the Free Software Foundation; either version 2 of the License, or
** (at your option) any later version.
**
** This program is distributed in the hope that it will be useful,
** but WITHOUT ANY WARRANTY; without even the implied warranty of
** MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
** GNU General Public License for more details.
**
** You should have received a copy of the GNU General Public License
** along with this program; if not, write to the Free Software
** Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA 02111-1307 USA
*/

#include "gazebo/common/Assert.hh"
#include "gazebo/common/Events.hh"
#include "ASVDynamicsPlugin.hh"

#include <gazebo/transport/transport.hh>
#include <gazebo/msgs/msgs.hh>
#include <math.h>

#include "gazebo/sensors/Sensor.hh"
#include "gazebo/sensors/SensorManager.hh"

using namespace gazebo;
using namespace std;

GZ_REGISTER_MODEL_PLUGIN(ASVDynamicsPlugin)

/////////////////////////////////////////////////
ASVDynamicsPlugin::ASVDynamicsPlugin()
    : fluidDensity(999.1026) // Density of liquid water at 1 atm pressure and 15 degrees Celsius.
{
}

/////////////////////////////////////////////////
void ASVDynamicsPlugin::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
{
//------------------------------------------------------------------------------------------
    
    GZ_ASSERT(_model != NULL, "Received NULL model pointer");
    GZ_ASSERT(_sdf != NULL, "Received NULL SDF pointer");
    
    physics::WorldPtr world = _model->GetWorld();
    GZ_ASSERT(world != NULL, "Model is in a NULL world");
    
//------------------------------------------------------------------------------------------
    
    this->model = _model;
    this->sdf = _sdf;

    this->fluidDensity                    = this->sdf->Get<double>("fluid_density");
    this->water_level                     = this->sdf->Get<double>("water_level");
    this->volume_of_boat                  = this->sdf->Get<double>("boat_volume");
    this->volume_of_boundingbox           = this->sdf->Get<double>("boat_boundingbox_volume");

    this->cBoundingBoxLength              =  this->sdf->Get<ignition::math::Vector3d>("boat_boundingbox_dimensions");
    this->center_of_volume_boat           =  this->sdf->Get<ignition::math::Vector3d>("boat_center_of_volume");

    this->dampingforcelinearcoefficients  =  this->sdf->Get<ignition::math::Vector3d>("damping_force_linear_coefficients");
    this->dampingtorquelinearcoefficients =  this->sdf->Get<ignition::math::Vector3d>("damping_torque_linear_coefficients");
    this->left_propeller_thrust           =  this->sdf->Get<double>("left_propeller_thrust");
    this->right_propeller_thrust          =  this->sdf->Get<double>("right_propeller_thrust");
    

    
    center_volume_displaced_fluid.X(center_of_volume_boat.X());
    center_volume_displaced_fluid.Y(center_of_volume_boat.Y());

    ///////////////////////////////////////////////////////////////////////////////////
    // Create the node
    this->node = transport::NodePtr(new transport::Node());
    this->node->Init(this->model->GetWorld()->Name());

    //////////////////////////////////////////////////////////////////////////////////
    // Create a topic name
    std::string topicname = "~/" + this->model->GetName() + "/thrustforce_cmd";

    //////////////////////////////////////////////////////////////////////////////////
    // Subscribe to the topic, and register a callback
    this->sub = this->node->Subscribe(topicname, &ASVDynamicsPlugin::OnMsg, this);
    
    
//------------------------------------------------------------------------------------------
    //Load surface mesh
    this->surfaceMesh =  common::MeshManager::Instance()->Load("/home/pawel/3rd_year_project/gazebo-asv/gazebo-asv/models/plane.dae");
    
    this->surfaceMesh->FillArrays(&this->surfaceMeshVertices, &this->surfaceMeshIndices);
    this->numSurfaceVertices = surfaceMesh->GetVertexCount();
    this->numSurfaceIndices = surfaceMesh->GetIndexCount();
    
    std::cout << "numSurfaceIndices: " << this->numSurfaceIndices << " numSurfaceVertices: " << this->numSurfaceVertices << std::endl;
    
    //////////////////////////////////////////////////////////////////////////////////
    //Load vehicle mesh
    //Currently, the downsampled version is unstable most likely due to inaccurate surface integration that is caused by lack of the triangle cutting algorithm
    this->vehicleMesh =  common::MeshManager::Instance()->Load("/home/pawel/3rd_year_project/gazebo-asv/gazebo-asv/models/asv_model/meshes/proto7_meters.dae");
    
    this->vehicleMesh->FillArrays(&this->vehicleMeshVertices, &this->vehicleMeshIndices);
    this->numVehicleVertices = vehicleMesh->GetVertexCount();
    this->numVehicleIndices = vehicleMesh->GetIndexCount();
    
    std::cout << "numVehicleIndices: " << this->numVehicleIndices << " numVehicleVertices: " << this->numVehicleVertices << std::endl;
    
//------------------------------------------------------------------------------------------
}
/////////////////////////////////////////////////
void ASVDynamicsPlugin::SetThrustForces(const double &_left_thrust, const double &_right_thrust)
{
    left_propeller_thrust = _left_thrust; right_propeller_thrust = _right_thrust;
}

/////////////////////////////////////////////////
void ASVDynamicsPlugin::OnMsg(ConstVector3dPtr &_msg)
{
    this->SetThrustForces(_msg->x(), _msg->y());
}

/////////////////////////////////////////////////
void ASVDynamicsPlugin::Init()
{
    gazebo::sensors::SensorManager *mgr = gazebo::sensors::SensorManager::Instance();

    this->updateConnection = event::Events::ConnectWorldUpdateBegin(std::bind(&ASVDynamicsPlugin::OnUpdate, this));

    link = model->GetLink("body");
    GZ_ASSERT(link != NULL, "The model has to have a body link");

    center_mass = link->GetInertial()->Pose().Pos();

    /*
     * Added mass coefficients: If link->GetInertial()->Mass() could give us a vector of masses (M_11, M_22, M_33),
     * then added masses could be simulated. But as Gazebo does not allow this, for now we do not simulate the added mass.
     */
    xu1 = -0.15f, yv1 = -38.2124, nr1 = -1.3597f;

    /*
     * Mass matrix of the rigid body.
     */
    m_rb    = link->GetInertial()->Mass(); //! link->GetInertial()->GetMass() - ignition::math::Vector3d(-xu1, -yv1, 0.0f);
    ixx_rb  = link->GetInertial()->IXX();
    iyy_rb  = link->GetInertial()->IYY();
    izz_rb  = link->GetInertial()->IZZ();
    ixy_rb  = link->GetInertial()->IXY();
    ixz_rb  = link->GetInertial()->IXZ();
    iyz_rb  = link->GetInertial()->IYZ();
    GZ_ASSERT(ixy_rb == 0.0f, "Moment ixy should be at 0");
    GZ_ASSERT(ixz_rb == 0.0f, "Moment ixz should be at 0");
    GZ_ASSERT(iyz_rb == 0.0f, "Moment iyz should be at 0");

//------------------------------------------------------------------------------------------
#ifdef DEBUG_MESSAGES
    std::cout << "m " << m_rb << std::endl;
    std::cout << "ixx " << ixx_rb << std::endl;
    std::cout << "iyy " << iyy_rb << std::endl;
    std::cout << "izz " << izz_rb << std::endl;
    std::cout << "CoM " << link->GetInertial()->GetPose() << std::endl;
#endif
//------------------------------------------------------------------------------------------
}

/////////////////////////////////////////////////
void ASVDynamicsPlugin::OnUpdate()
{
    //Initialize the kinematics variables
	ignition::math::Vector3d pos = link->WorldPose().Pos();
	ignition::math::Quaterniond rot = link->WorldPose().Rot();
    ignition::math::Vector3d vel = link->WorldLinearVel();
    ignition::math::Vector3d omega = link->WorldAngularVel();

    //Define an array to store a position of every vertex on the mesh
    float vehicleVertices[3*numVehicleVertices];

    //Update every vertex on the mesh by translating and rotating the initial mesh cloud according to the pose of the vehicle
	for(int i=0; i<numVehicleVertices; i++){
        ignition::math::Vector3d vertex(vehicleMeshVertices[i*3+0], vehicleMeshVertices[i*3+1], vehicleMeshVertices[i*3+2]);
        vertex = vertex + pos;
        vertex = rot.RotateVector(vertex);
        //vertex = vertex + pos;
        vehicleVertices[i*3+0] = vertex.X();
        vehicleVertices[i*3+1] = vertex.Y();
        vehicleVertices[i*3+2] = vertex.Z();

	}

	cout<<"Random point: "<<vehicleVertices[0]<<" "<<vehicleVertices[1]<<" "<<vehicleVertices[2]<<endl<<endl;
    cout<<"Pose: \n"<<pos<<endl;

    //Define hydrostatic force and torque that capture damping
	ignition::math::Vector3d hydrostatic_force(0.0f, 0.0f, 0.0f);
    ignition::math::Vector3d hydrostatic_torque(0.0f, 0.0f, 0.0f);
	
	//Number of total and submerged facets
	float S_submerged = 0;
	float S_total = 0;


	for(int i=0; i<numVehicleIndices; i+=3){
		
		
		//Triangle coordinates
		float x1 = vehicleVertices[vehicleMeshIndices[i]*3+0];
		float y1 = vehicleVertices[vehicleMeshIndices[i]*3+1];
		float z1 = vehicleVertices[vehicleMeshIndices[i]*3+2];
		
		float x2 = vehicleVertices[vehicleMeshIndices[i+1]*3+0];
		float y2 = vehicleVertices[vehicleMeshIndices[i+1]*3+1];
		float z2 = vehicleVertices[vehicleMeshIndices[i+1]*3+2];
		
		float x3 = vehicleVertices[vehicleMeshIndices[i+2]*3+0];
		float y3 = vehicleVertices[vehicleMeshIndices[i+2]*3+1];
		float z3 = vehicleVertices[vehicleMeshIndices[i+2]*3+2];
		
		//Calculate the area of the triangle
		float dS = 0.5*sqrt((x2*y3-x3*y2)*(x2*y3-x3*y2)+(x3*y1-x1*y3)*(x3*y1-x1*y3)+(x1*y2-x2*y1)*(x1*y2-x2*y1));
		S_total+=dS;

		//For every fully submerged triangle
        //This will have to improved to take into account triangle cutting algorithm
        //Otherwise, for low quiality meshes the integration is highly inaccurate
		if(z1-water_level < 0 && z2-water_level < 0 && z3-water_level < 0){

			//Accumulate all the submerged triangles
			S_submerged+=dS;
			
			//Calculate the normal of the triangle
			float a1 = x2-x1;
			float a2 = y2-y1;
			float a3 = z2-z1;
		
			float b1 = x3-x1;
			float b2 = y3-y1;
			float b3 = z3-z1;
			ignition::math::Vector3d normal((a2*b3-a3*b2), -(a1*b3-a3*b1), (a1*b2-a2*b1));
            normal = normal.Normalized();

            //Find the centre of the triangle
            ignition::math::Vector3d centre((x1+x2+x3)/3, (y1+y2+y3)/3, (z1+z2+z3)/3);

            //Find the vector from the CoG to the centre of the triangle
            ignition::math::Vector3d GC = pos - centre;

            //Compute velocity of the triangle
            ignition::math::Vector3d velocity = vel+omega.Cross(GC);

            //Define the cosine of the angle
            double cos_theta = velocity.Normalized().Dot(normal);

			//Calculate the height difference between the water level and the centre of the triangle
			float h_centre = ((z1+z2+z3)/3)-water_level;

			//Calculate the triangle contribution to the hydrostatic force
            //Currently, neglects the x and y components. Also, undesired oscilations appear.
			ignition::math::Vector3d dBouyancy = -fluidDensity*(model->GetWorld()->Gravity())*h_centre*normal*dS;

            //Calculate the viscous resistance contribution of the triangle
            //ignition::math::Vector3d dViscousResistance = 0.5*fluidDensity*;

            //Define the drag coefficients and calculate the pressure drag
            double Cpd1=10, Cpd2=10.0, Csd1=10.0, Csd2=10.0;
            double V=velocity.Length(),Vr = 1;
            ignition::math::Vector3d dPressureDrag(0.0f, 0.0f, 0.0f);
            if(cos_theta>0)
                dPressureDrag = normal*(Cpd1*V/Vr+Cpd2*(V/Vr)*(V/Vr))*dS*(cos_theta);
            else
                dPressureDrag = -normal*(Csd1*V/Vr+Csd2*(V/Vr)*(V/Vr))*dS*(cos_theta);

            //Add the contribution of all the forces
            ignition::math::Vector3d dF = dBouyancy;

			link->AddForceAtWorldPosition(dF, centre);
			
		}
		
	}

	//Define dumping coefficients
	float C_damp = 1000;
    float C_rotational_damp = 200;

	//Calculate drag force and torque
	ignition::math::Vector3d drag = (link->WorldLinearVel())*(-(C_damp*S_submerged/S_total));
    ignition::math::Vector3d rotationalDrag = (link->WorldAngularVel())*(-(C_rotational_damp*S_submerged/S_total));
	hydrostatic_force += drag;
    //hydrostatic_torque += rotationalDrag;

    //Apply the hydrostatic force and torque to the model
	link->AddForce(hydrostatic_force);
    //link->AddTorque(hydrostatic_torque);

}
