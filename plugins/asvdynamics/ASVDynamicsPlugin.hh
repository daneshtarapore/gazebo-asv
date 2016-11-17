/*
** ASVDynamicsPlugin.hh
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

#ifndef _GAZEBO_ASV_DYNAMICS_PLUGIN_HH_
#define _GAZEBO_ASV_DYNAMICS_PLUGIN_HH_

#include <map>
#include "gazebo/common/Event.hh"
#include "gazebo/common/Plugin.hh"
#include "gazebo/math/Vector3.hh"
#include "gazebo/physics/physics.hh"

//#define DEBUG_MESSAGES

namespace gazebo
{
//  /// \brief A class for storing the volume properties of a link.
//  class VolumeProperties
//  {
//    /// \brief Default constructor.
//    public: VolumeProperties() : volume(0) {}

//    /// \brief Center of volume in the link frame.
//    public: math::Vector3 cov;

//    /// \brief Volume of this link.
//    public: double volume;
//  };

  /// \brief A plugin that simulates all the dynamics of an ASV
  /// All SDF parameters are mandatory.
  /// <fluid_density> sets the density of the fluid that surrounds the ASV object. The fluid is assumed to be water
  /// <water_level> the height of water in the world
  /// <boat_volume> the volume of the ASV (can be obtained from Meshlab)
  /// <boat_boundingbox_volume> the volume of the bounding box encompassing the ASV
  /// <boat_boundingbox_dimensions> the dimensions of the bounding box encompassing the ASV
  /// <boat_center_of_volume> the center of volume of the ASV (can be obtained from Meshlab)
  /// <damping_force_linear_coefficients> the linear damping coefficients for damping force
  /// <damping_torque_linear_coefficients> the linear damping coefficients for damping torque

  class GAZEBO_VISIBLE ASVDynamicsPlugin : public ModelPlugin
  {
    /// \brief Constructor.
    public: ASVDynamicsPlugin();

    /// \brief Read the model SDF to compute volume and center of volume for
    /// each link, and store those properties in volPropsMap.
    public: virtual void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf);

    // Documentation inherited
    public: virtual void Init();

    //protected: math::Box cBoundingBox;

    protected: math::Vector3 center_volume_displaced_fluid;

    /// \brief Callback for World Update events.
    protected: virtual void OnUpdate();

    /// \brief Connection to World Update events.
    protected: event::ConnectionPtr updateConnection;

    /// \brief Pointer to model containing the plugin.
    protected: physics::ModelPtr model;

      /// \brief Pointer to link contained in the model.
    protected: physics::LinkPtr link;

    /// \brief Pointer to the plugin SDF.
    protected: sdf::ElementPtr sdf;

    /// \brief Center of mass of boat
    protected: math::Vector3 center_mass;

    /// \brief Volume of boat (courtesy of Meshlab), and volume of encompassing bounding box
    protected: double volume_of_boat, volume_of_boundingbox;

    /// \brief Dimensions of bounding box encompassing boat
    protected: math::Vector3 cBoundingBoxLength;

    /// \brief Center of volume of boat
    protected: math::Vector3 center_of_volume_boat;

    /// \brief Added mass coefficients. If link->GetInertial()->GetMass() could give us a vector of masses (M_11, M_22, M_33), then added masses could be simulated. But as Gazebo does not allow this, for now we do not simulate the added mass. So please ignore these coefficients for now.
    protected: double xu1, yv1, nr1;

    /// \brief Mass matrix of the rigid body
    protected: double m_rb, ixx_rb, iyy_rb, izz_rb, ixy_rb, ixz_rb, iyz_rb;

    /// \brief Damping linear coefficients
    protected: math::Vector3 dampingforcelinearcoefficients, dampingtorquelinearcoefficients;

    /// \brief Level of water in world
    protected: double water_level;

    /// \brief The density of the fluid in which the object is submerged in
    /// kg/m^3. Defaults to 1000, the fluid density of water.
    protected: double fluidDensity;
  };
}

#endif
