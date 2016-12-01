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
  this->updateConnection = this->parentSensor->ConnectUpdated(std::bind(&GpsPlugin::OnUpdate, this));

  //! see http://www.boost.org/doc/libs/1_42_0/libs/bind/bind.html#with_functions

#if GAZEBO_MAJOR_VERSION > 6
      this->parentSensor->Noise(sensors::GPS_POSITION_LATITUDE_NOISE_METERS)->SetCustomNoiseCallback(
        boost::bind(&GpsPlugin::Perturb,this,0,_1));
      this->parentSensor->Noise(sensors::GPS_POSITION_LONGITUDE_NOISE_METERS)->SetCustomNoiseCallback(
        boost::bind(&GpsPlugin::Perturb,this,1,_1));
      this->parentSensor->Noise(sensors::GPS_POSITION_ALTITUDE_NOISE_METERS)->SetCustomNoiseCallback(
        boost::bind(&GpsPlugin::Perturb,this,2,_1));
      this->parentSensor->Noise(sensors::GPS_VELOCITY_LATITUDE_NOISE_METERS)->SetCustomNoiseCallback(
        boost::bind(&GpsPlugin::Perturb,this,3,_1));
      this->parentSensor->Noise(sensors::GPS_VELOCITY_LONGITUDE_NOISE_METERS)->SetCustomNoiseCallback(
        boost::bind(&GpsPlugin::Perturb,this,4,_1));
      this->parentSensor->Noise(sensors::GPS_VELOCITY_ALTITUDE_NOISE_METERS)->SetCustomNoiseCallback(
        boost::bind(&GpsPlugin::Perturb,this,5,_1));
#else
      this->parentSensor->GetNoise(sensors::GPS_POSITION_LATITUDE_NOISE_METERS)->SetCustomNoiseCallback(
        boost::bind(&GpsPlugin::Perturb,this,0,_1));
      this->parentSensor->GetNoise(sensors::GPS_POSITION_LONGITUDE_NOISE_METERS)->SetCustomNoiseCallback(
        boost::bind(&GpsPlugin::Perturb,this,1,_1));
      this->parentSensor->GetNoise(sensors::GPS_POSITION_ALTITUDE_NOISE_METERS)->SetCustomNoiseCallback(
        boost::bind(&GpsPlugin::Perturb,this,2,_1));
      this->parentSensor->GetNoise(sensors::GPS_VELOCITY_LATITUDE_NOISE_METERS)->SetCustomNoiseCallback(
        boost::bind(&GpsPlugin::Perturb,this,3,_1));
      this->parentSensor->GetNoise(sensors::GPS_VELOCITY_LONGITUDE_NOISE_METERS)->SetCustomNoiseCallback(
        boost::bind(&GpsPlugin::Perturb,this,4,_1));
      this->parentSensor->GetNoise(sensors::GPS_VELOCITY_ALTITUDE_NOISE_METERS)->SetCustomNoiseCallback(
        boost::bind(&GpsPlugin::Perturb,this,5,_1));
#endif


  // Make sure the parent sensor is active.
  this->parentSensor->SetActive(true);
}

/////////////////////////////////////////////////
void GpsPlugin::OnUpdate()
{

}

/////////////////////////////////////////////////
// Perturb the value with the actual error from the system
double GpsPlugin::Perturb(int i, double val)
{
  return val* 0.0f;
}
