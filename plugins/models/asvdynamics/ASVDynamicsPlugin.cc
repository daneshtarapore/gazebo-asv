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

#include "gazebo/sensors/Sensor.hh"
#include "gazebo/sensors/SensorManager.hh"

using namespace gazebo;

GZ_REGISTER_MODEL_PLUGIN(ASVDynamicsPlugin)

/////////////////////////////////////////////////
ASVDynamicsPlugin::ASVDynamicsPlugin()
    : fluidDensity(999.1026) // Density of liquid water at 1 atm pressure and 15 degrees Celsius.
{
}

/////////////////////////////////////////////////
void ASVDynamicsPlugin::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
{
    GZ_ASSERT(_model != NULL, "Received NULL model pointer");
    this->model = _model;
    physics::WorldPtr world = _model->GetWorld();
    GZ_ASSERT(world != NULL, "Model is in a NULL world");

    GZ_ASSERT(_sdf != NULL, "Received NULL SDF pointer");
    this->sdf = _sdf;

    //if (this->sdf->HasElement("fluid_density"))

    this->fluidDensity                    = this->sdf->Get<double>("fluid_density");
    this->water_level                     = this->sdf->Get<double>("water_level");
    this->volume_of_boat                  = this->sdf->Get<double>("boat_volume");
    this->volume_of_boundingbox           = this->sdf->Get<double>("boat_boundingbox_volume");

    this->cBoundingBoxLength              =  this->sdf->Get<math::Vector3>("boat_boundingbox_dimensions");
    this->center_of_volume_boat           =  this->sdf->Get<math::Vector3>("boat_center_of_volume");
    center_volume_displaced_fluid.x = center_of_volume_boat.x;
    center_volume_displaced_fluid.y = center_of_volume_boat.y;
    this->dampingforcelinearcoefficients  =  this->sdf->Get<math::Vector3>("damping_force_linear_coefficients");
    this->dampingtorquelinearcoefficients =  this->sdf->Get<math::Vector3>("damping_torque_linear_coefficients");
    this->left_propeller_thrust           =  this->sdf->Get<double>("left_propeller_thrust");
    this->right_propeller_thrust          =  this->sdf->Get<double>("right_propeller_thrust");


    // Create the node
    this->node = transport::NodePtr(new transport::Node());
    this->node->Init(this->model->GetWorld()->GetName());

    // Create a topic name
    std::string topicname = "~/" + this->model->GetName() + "/thrustforce_cmd";

    // Subscribe to the topic, and register a callback
    this->sub = this->node->Subscribe(topicname, &ASVDynamicsPlugin::OnMsg, this);


    //std::cout << "dampingtorquelinearcoefficients " << dampingtorquelinearcoefficients << std::endl;


    // Get "center of volume" and "volume" that were inputted in SDF
    // SDF input is recommended for mesh or polylines collision shapes
    /*if (this->sdf->HasElement("link"))
    {
        for (sdf::ElementPtr linkElem = this->sdf->GetElement("link"); linkElem;
             linkElem = linkElem->GetNextElement("link"))
        {
            int id = -1;
            std::string name = "";
            if (linkElem->HasAttribute("name"))
            {
                name = linkElem->Get<std::string>("name");
                physics::LinkPtr link =
                        this->model->GetLink(name);
                if (!link)
                {
                    gzwarn << "Specified link [" << name << "] not found." << std::endl;
                    continue;
                }
                id = link->GetId();
            }
            else
            {
                gzwarn << "Required attribute name missing from link [" << name
                       << "] in ASVDynamicsPlugin SDF" << std::endl;
                // Exit if we didn't set ID
                continue;
            }

            if (this->volPropsMap.count(id) != 0)
            {
                gzwarn << "Properties for link [" << name << "] already set, skipping "
                       << "second property block" << std::endl;
            }

            if (linkElem->HasElement("center_of_volume"))
            {
                math::Vector3 cov =
                        linkElem->GetElement("center_of_volume")->Get<math::Vector3>();
                this->volPropsMap[id].cov = cov;
            }
            else
            {
                gzwarn << "Required element center_of_volume missing from link ["
                       << name
                       << "] in ASVDynamicsPlugin SDF" << std::endl;
                continue;
            }

            if (linkElem->HasElement("volume"))
            {
                double volume = linkElem->GetElement("volume")->Get<double>();
                if (volume <= 0)
                {
                    gzwarn << "Nonpositive volume specified in ASVDynamicsPlugin!"
                           << std::endl;
                    // Remove the element from the map since it's invalid.
                    this->volPropsMap.erase(id);
                    continue;
                }
                this->volPropsMap[id].volume = volume;
            }
            else
            {
                gzwarn << "Required element volume missing from element link [" << name
                       << "] in ASVDynamicsPlugin SDF" << std::endl;
                continue;
            }
        }
    }*/

    // For links the user didn't input, precompute the center of volume and
    // density. This will be accurate for simple shapes.
    /*for (auto link : this->model->GetLinks())
    {
        int id = link->GetId();
        if (this->volPropsMap.find(id) == this->volPropsMap.end())
        {
            double volumeSum = 0;
            math::Vector3 weightedPosSum = math::Vector3::Zero;

            // The center of volume of the link is a weighted average over the pose
            // of each collision shape, where the weight is the volume of the shape
            int count =0;
            for (auto collision : link->GetCollisions())
            {
                double volume = collision->GetShape()->ComputeVolume();
                volumeSum += volume;
                weightedPosSum += volume*collision->GetWorldPose().pos;
                count++;

                cBoundingBox  = collision->GetCollisionBoundingBox();
                std::cout << "count " << count << " cbb center " << cBoundingBox.GetCenter() << " cbb size " << cBoundingBox.GetSize() << std::endl;
            }
            // Subtract the center of volume into the link frame.
            this->volPropsMap[id].cov =
                    weightedPosSum/volumeSum - link->GetWorldPose().pos;
            this->volPropsMap[id].volume = volumeSum;
        }
    }*/
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

    center_mass = link->GetInertial()->GetPose().pos;

    /*
     * Added mass coefficients: If link->GetInertial()->GetMass() could give us a vector of masses (M_11, M_22, M_33), 
     * then added masses could be simulated. But as Gazebo does not allow this, for now we do not simulate the added mass.
     */
    xu1 = -0.15f, yv1 = -38.2124, nr1 = -1.3597f;

    /*
     * Mass matrix of the rigid body.
     */
    m_rb    = link->GetInertial()->GetMass(); //! link->GetInertial()->GetMass() - math::Vector3(-xu1, -yv1, 0.0f);
    ixx_rb  = link->GetInertial()->GetIXX();
    iyy_rb  = link->GetInertial()->GetIYY();
    izz_rb  = link->GetInertial()->GetIZZ();
    ixy_rb  = link->GetInertial()->GetIXY();
    ixz_rb  = link->GetInertial()->GetIXZ();
    iyz_rb  = link->GetInertial()->GetIYZ();
    GZ_ASSERT(ixy_rb == 0.0f, "Moment ixy should be at 0");
    GZ_ASSERT(ixz_rb == 0.0f, "Moment ixz should be at 0");
    GZ_ASSERT(iyz_rb == 0.0f, "Moment iyz should be at 0");

#ifdef DEBUG_MESSAGES
    std::cout << "m " << m_rb << std::endl;
    std::cout << "ixx " << ixx_rb << std::endl;
    std::cout << "iyy " << iyy_rb << std::endl;
    std::cout << "izz " << izz_rb << std::endl;
    std::cout << "CoM " << link->GetInertial()->GetPose() << std::endl;
#endif
}

/////////////////////////////////////////////////
void ASVDynamicsPlugin::OnUpdate()
{
    //for (auto link : this->model->GetLinks())
    //{
        math::Vector3 total_force(0.0f, 0.0f, 0.0f), total_torque(0.0f, 0.0f, 0.0f);

        /*
         * Mass matrix of the rigid body.
         */
        /*m_rb    = link->GetInertial()->GetMass(); //! link->GetInertial()->GetMass() - math::Vector3(-xu1, -yv1, 0.0f);
        ixx_rb  = link->GetInertial()->GetIXX();
        iyy_rb  = link->GetInertial()->GetIYY();
        izz_rb  = link->GetInertial()->GetIZZ();
        ixy_rb  = link->GetInertial()->GetIXY();
        ixz_rb  = link->GetInertial()->GetIXZ();
        iyz_rb  = link->GetInertial()->GetIYZ();
        GZ_ASSERT(ixy_rb == 0.0f, "Moment ixy should be at 0");
        GZ_ASSERT(ixz_rb == 0.0f, "Moment ixz should be at 0");
        GZ_ASSERT(iyz_rb == 0.0f, "Moment iyz should be at 0");*/





        //double cBoundingBoxLength.x = 0.653f;
        //double cBoundingBoxLength.y = 0.386f;
        //double cBoundingBoxLength.z = 0.10f;


        //double volume_of_boat = 0.006994f; // volume in m^3

        double volume_displaced_fluid, boundingbox_depth_underwater;
        //double center_of_volume_boat.x = -0.04f; //-0.0485f;
        //double center_of_volume_boat.y =  0.00f;
        //double center_of_volume_boat.z =  0.07f;

        //math::Vector3 center_mass = math::Vector3(-0.04f, 0.0f, 0.067f);


        math::Pose linkFrame = link->GetWorldPose();
        math::Vector3 pos = linkFrame.pos;
        math::Vector3 rot_euler = linkFrame.rot.GetAsEuler();


        //float water_level = 10.0f; //m
        //volume_displaced_fluid = 0.0f;
        if(pos.z > water_level) // pos.z is the coordinate of the lower plane of the bounding box
        {
            // Body is completely above water surface
            volume_displaced_fluid = 0.0f;
#ifdef DEBUG_MESSAGES
            std::cout << "Body is completely above water surface " << std::endl;
#endif
        }
        else if((pos.z + cBoundingBoxLength.z) < water_level)
        {
            // Body is completely submerged under water
            boundingbox_depth_underwater = cBoundingBoxLength.z;
            volume_displaced_fluid = volume_of_boundingbox; //(cBoundingBoxLength.x * cBoundingBoxLength.y * cBoundingBoxLength.z);
            //center_volume_displaced_fluid.x = center_of_volume_boat.x;
            //center_volume_displaced_fluid.y = center_of_volume_boat.y;
            center_volume_displaced_fluid.z = center_of_volume_boat.z;
#ifdef DEBUG_MESSAGES
            std::cout << "Body is completely submerged under water " << std::endl;
#endif
        }
        else
        {
            boundingbox_depth_underwater = water_level - pos.z;
            volume_displaced_fluid = cBoundingBoxLength.x * cBoundingBoxLength.y * boundingbox_depth_underwater;
            //center_volume_displaced_fluid.x = center_of_volume_boat.x;
            //center_volume_displaced_fluid.y = center_of_volume_boat.y;
            center_volume_displaced_fluid.z = boundingbox_depth_underwater/2.0f;
#ifdef DEBUG_MESSAGES
            std::cout << "Body is partially submerged under water " << std::endl;
#endif
        }

        if(volume_displaced_fluid == 0.0f)
            return;


        //if(volume_displaced_fluid > 0.0f)
        //{
            //std::cout << " volumeProperties.volume " << volumeProperties.volume << std::endl;
            //std::cout << " BB.volume               " << (cBoundingBox.GetXLength() * cBoundingBox.GetYLength() * cBoundingBox.GetZLength()) << std::endl;
            //! 0.006994f m^2 is the volume of the boat (courtesy Meshlabs)
            volume_displaced_fluid *= volume_of_boat / volume_of_boundingbox; //(cBoundingBoxLength.x * cBoundingBoxLength.y * cBoundingBoxLength.z);


            // By Archimedes' principle,
            // buoyancy = -(mass*gravity)*fluid_density/object_density
            // object_density = mass/volume, so the mass term cancels.
            // Therefore,
            math::Vector3 buoyancy_force = - volume_displaced_fluid * this->fluidDensity * this->model->GetWorld()->Gravity();

            // rotate buoyancy into the link frame
            /*math::Vector3 buoyancyforce_bodyframe = math::Vector3(-buoyancy_force.z * sin(rot_euler.y),
                                                                  buoyancy_force.z * cos(rot_euler.y) * sin(rot_euler.x),
                                                                  buoyancy_force.z * cos(rot_euler.y) * cos(rot_euler.x));*/
            math::Vector3 buoyancyforce_bodyframe = linkFrame.rot.GetInverse().RotateVector(buoyancy_force);
            //!link->AddLinkForce(buoyancyforce_bodyframe, center_volume_displaced_fluid);
            total_force += buoyancyforce_bodyframe;

            // torque due to buoyancy forces
            math::Vector3 buoyancytorque_bodyframe;
            buoyancytorque_bodyframe.x = (this->fluidDensity * this->model->GetWorld()->Gravity().Z() * volume_displaced_fluid *
                                         ((1.0f/12.0f)*(cBoundingBoxLength.y*cBoundingBoxLength.y/boundingbox_depth_underwater) -
                                          (center_mass.z - center_volume_displaced_fluid.z))*sin(rot_euler.x));

            buoyancytorque_bodyframe.y = (this->fluidDensity * this->model->GetWorld()->Gravity().Z() * volume_displaced_fluid *
                                         ((1.0f/12.0f)*(cBoundingBoxLength.x*cBoundingBoxLength.x/boundingbox_depth_underwater) -
                                          (center_mass.z - center_volume_displaced_fluid.z))*sin(rot_euler.y));

            buoyancytorque_bodyframe.z = 0.0f;

            //!link->AddTorque(buoyancytorque_bodyframe);
            total_torque += buoyancytorque_bodyframe;




            /*
             *  Linear damping forces and torques
             */

            /*double dampingforcelinearcoefficients.x =  10.0f;
            double dampingforcelinearcoefficients.y =  10.0f;
            double dampingforcelinearcoefficients.z =  10.0f;

            double dampingtorquelinearcoefficients.x = 10.0f;
            double dampingtorquelinearcoefficients.y = 10.0f;
            double dampingtorquelinearcoefficients.z = 10.0f;*/





            math::Vector3 dampingForce = math::Vector3(dampingforcelinearcoefficients.x * link->GetRelativeLinearVel().x,
                                                       dampingforcelinearcoefficients.y * link->GetRelativeLinearVel().y,
                                                       dampingforcelinearcoefficients.z * link->GetRelativeLinearVel().z);
            //math::Vector3 rotateddampingForce = link->GetWorldPose().rot.GetInverse().RotateVector(dampingForce);
            //link->AddLinkForce(rotateddampingForce, center_mass);
            //std::cout << " rotated dampingforce " << rotateddampingForce << std::endl;
            //!link->AddLinkForce(dampingForce, center_mass);
            total_force += -dampingForce;




            math::Vector3 dampingTorque = math::Vector3(dampingtorquelinearcoefficients.x * link->GetRelativeAngularVel().x,
                                                        dampingtorquelinearcoefficients.y * link->GetRelativeAngularVel().y,
                                                        dampingtorquelinearcoefficients.z * link->GetRelativeAngularVel().z);



            //!link->AddTorque(dampingTorque);
            total_torque += -dampingTorque;




            /*
             *  Coriolis forces acting on the mass rigid body
             */
            double u = 0.0f, v = 0.0f, w = 0.0f, p = 0.0f, q = 0.0f, r = 0.0f;
            u = link->GetRelativeLinearVel().x;  v = link->GetRelativeLinearVel().y;  w = link->GetRelativeLinearVel().z;
            p = link->GetRelativeAngularVel().x; q = link->GetRelativeAngularVel().y; r = link->GetRelativeAngularVel().z;
            math::Vector3 coriolis_force  = math::Vector3(-m_rb*r*v + m_rb*q*w, m_rb*r*u - m_rb*p*w, -m_rb*q*u + m_rb*p*v);
            math::Vector3 coriolis_torque = math::Vector3(-q*r*iyy_rb + q*r*izz_rb, p*r*ixx_rb - p*r*izz_rb, -p*q*ixx_rb + p*q*iyy_rb);
            total_force  += -coriolis_force;
            total_torque += -coriolis_torque;




            /*
             * Coriolis forces acting on the ADDED mass rigid body
             */

            /*
            math::Vector3 coriolis_force_added   =  -math::Vector3(r*v*Yv1, r*u*Xu1, q*u*Xu1 - p*v*Yv1);
            math::Vector3 coriolis_torque_added  =  -math::Vector3(-q*r*Nr1 + v*w*Yv1, p*r*Nr1 - u*w*Xu1, u*v*Xu1 - u*v*Yv1);
            total_force  += -coriolis_force_added;
            total_torque += -coriolis_torque_added;
            */



            /*
             *  Thrust forces
             */
            //if(model->GetWorld()->GetEntity("surfacevehicle_1")->IsSelected()) // Does not work
            /*if(model->GetName().compare("surfacevehicle_0"))
            {
                left_propeller_thrust = 0.0f; right_propeller_thrust = 0.0f;
            }*/

            math::Vector3 left_propeller_position =  math::Vector3(-0.029f, -0.0975f, 0.036f);
            math::Vector3 right_propeller_position = math::Vector3(-0.029f, +0.0975f, 0.036f);

            math::Vector3 left_propeller_thrust_force  = math::Vector3(left_propeller_thrust, 0.0f, 0.0f);
            math::Vector3 right_propeller_thrust_force = math::Vector3(right_propeller_thrust, 0.0f, 0.0f);

            //!link->AddLinkForce(left_propeller_thrust_force, left_propeller_position);
            //!link->AddLinkForce(right_propeller_thrust_force, right_propeller_position);

            total_force  += (left_propeller_thrust_force + right_propeller_thrust_force);

            math::Vector3 left_propeller_thrust_torque, right_propeller_thrust_torque;
            left_propeller_thrust_torque  = left_propeller_thrust_force.Cross(left_propeller_position - center_mass);
            right_propeller_thrust_torque = right_propeller_thrust_force.Cross(right_propeller_position - center_mass);

            left_propeller_thrust_torque.y *= -1.0f; right_propeller_thrust_torque.y *= -1.0f;  // sign changed in accordance with Gazebo coordinate frame

            total_torque += left_propeller_thrust_torque;
            total_torque += right_propeller_thrust_torque;


        //}



        link->AddRelativeForce(total_force);
        link->AddRelativeTorque(total_torque);
        //link->SetForce(total_force); // SetForce is the only apply force function that works for Bullet, but I think it the force vector is specified in the world frame
        //link->SetTorque(total_torque);

    //}



#ifdef DEBUG_MESSAGES
            std::cout << " pos " << pos << std::endl;
            std::cout << " rot " << rot_euler << std::endl;
            std::cout << "approx. volume_displaced_fluid " << volume_displaced_fluid << std::endl;
            std::cout << "buoyancy_force (world frame)" << buoyancy_force << " with g=" << this->model->GetWorld()->Gravity().Z() << std::endl;
            std::cout << "buoyancyforce_bodyframe " << buoyancyforce_bodyframe << " center_volume_displaced_fluid " << center_volume_displaced_fluid << std::endl;
            std::cout << "buoyancytorque_bodyframe " << buoyancytorque_bodyframe << std::endl;
            math::Vector3 linearvelocity_worldframe = link->GetWorldLinearVel();
            std::cout << "linearvelocity_worldframe " << linearvelocity_worldframe << std::endl;
            std::cout << "linearvelocity relative " << link->GetRelativeLinearVel()  << std::endl;
            std::cout << " dampingforce " << dampingForce << std::endl;
            std::cout << " angularvelocity_worldframe " << link->GetWorldAngularVel() << std::endl;
            std::cout << " angularvelocity relative " << link->GetRelativeAngularVel() << std::endl;
            std::cout << " dampingTorque " << dampingTorque << std::endl;
            std::cout << "Relative torque " << link->GetRelativeTorque() << std::endl;
            std::cout << "coriolis force " << coriolis_force << std::endl;
            std::cout << "coriolis torque" << coriolis_torque << std::endl;
            std::cout << " total_thrust_force " << (left_propeller_thrust_force + right_propeller_thrust_force) << std::endl;
            std::cout << "left_propeller_torque " <<  left_propeller_thrust_torque << std::endl;
            std::cout << "right_propeller_torque " << right_propeller_thrust_torque << std::endl;
            std::cout << "Total force " << total_force << std::endl;
            std::cout << "Total torque " << total_torque << std::endl;
#endif
}
