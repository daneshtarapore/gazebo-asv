#ifndef _GAZEBO_GPS_PLUGIN_HH_
#define _GAZEBO_GPS_PLUGIN_HH_

#include <string>

#include <gazebo/gazebo.hh>
#include <gazebo/sensors/sensors.hh>

namespace gazebo
{
  /// \brief An example plugin for a contact sensor.
  class GpsPlugin : public SensorPlugin
  {
    /// \brief Constructor.
    public: GpsPlugin();

    /// \brief Destructor.
    public: virtual ~GpsPlugin();

    /// \brief Load the sensor plugin.
    /// \param[in] _sensor Pointer to the sensor that loaded this plugin.
    /// \param[in] _sdf SDF element that describes the plugin.
    public: virtual void Load(sensors::SensorPtr _sensor, sdf::ElementPtr _sdf);

    /// \brief Callback that receives the GPS sensor's update signal.
    private: virtual void OnUpdate();

    /// \brief Pointer to the GPS sensor
    private: sensors::GpsSensorPtr parentSensor;

    /// \brief Connection that maintains a link between the GPS sensor's
    /// updated signal and the OnUpdate callback.
    private: event::ConnectionPtr updateConnection;


    //private: double Perturb(int i, double val);

  };
}
#endif
