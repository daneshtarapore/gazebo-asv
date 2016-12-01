#include "GpsPlugin.hh"

using namespace gazebo;
GZ_REGISTER_SENSOR_PLUGIN(GpsPlugin)

/////////////////////////////////////////////////
GpsPlugin::GpsPlugin() : SensorPlugin()
{
}

/////////////////////////////////////////////////
GpsPlugin::~GpsPlugin()
{
}

/////////////////////////////////////////////////
void GpsPlugin::Load(sensors::SensorPtr _sensor, sdf::ElementPtr /*_sdf*/)
{
  // Get the parent sensor.
  this->parentSensor =
    std::dynamic_pointer_cast<sensors::GpsSensor>(_sensor);

  // Make sure the parent sensor is valid.
  if (!this->parentSensor)
  {
    gzerr << "GpsPlugin requires a GpsSensor.\n";
    return;
  }

  // Connect to the sensor update event.
  this->updateConnection = this->parentSensor->ConnectUpdated(
      std::bind(&GpsPlugin::OnUpdate, this));

  // Make sure the parent sensor is active.
  this->parentSensor->SetActive(true);
}

/////////////////////////////////////////////////
void GpsPlugin::OnUpdate()
{

}
