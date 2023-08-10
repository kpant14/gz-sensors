// /*
//  * Copyright (C) 2021 Open Source Robotics Foundation
//  *
//  * Licensed under the Apache License, Version 2.0 (the "License");
//  * you may not use this file except in compliance with the License.
//  * You may obtain a copy of the License at
//  *
//  *     http://www.apache.org/licenses/LICENSE-2.0
//  *
//  * Unless required by applicable law or agreed to in writing, software
//  * distributed under the License is distributed on an "AS IS" BASIS,
//  * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
//  * See the License for the specific language governing permissions and
//  * limitations under the License.
//  *
// */

#ifdef _WIN32
#pragma warning(push)
#pragma warning(disable: 4005)
#pragma warning(disable: 4251)
#endif
#include <gz/msgs/navsat.pb.h>
#ifdef _WIN32
#pragma warning(pop)
#endif

#include <gz/common/Profiler.hh>
#include <gz/math/Angle.hh>
#include <gz/math/Vector3.hh>
#include <gz/msgs/Utility.hh>
#include <gz/transport/Node.hh>

#include "gz/sensors/NavSatMultipathSensor.hh"
#include "gz/sensors/Noise.hh"
#include "gz/sensors/SensorFactory.hh"
#include "gz/sensors/SensorTypes.hh"

using namespace gz;
using namespace sensors;

/// \brief Private data for NavSat
class gz::sensors::NavSatMultipathPrivate
{
  /// \brief Node to create publisher
  public: transport::Node node;

  /// \brief To publish NavSat messages.
  public: transport::Node::Publisher pub;

  /// \brief True if Load() has been called and was successful
  public: bool loaded = false;

  /// \brief Latitude angle
  public: math::Angle latitude;

  /// \brief Longitude angle
  public: math::Angle longitude;

  /// \brief Altitude
  public: double altitude = 0.0;

  /// \brief Horizontal Ray Count
  public: int horizontalRayCount = 360;

  /// \brief Vertical Ray Count
  public: int verticalRayCount = 90;

  /// \brief Maximum Laser Range
  public: double maxRange = 100;

  /// \brief Minimum Laser Range
  public: double minRange = 1;

  /// \brief Velocity in ENU frame.
  public: math::Vector3d velocity;

  /// \brief Noise added to sensor data
  public: std::unordered_map<SensorNoiseType, NoisePtr> noises;

  // For GPU Laser sensor
  /// \brief Rendering camera
  public: gz::rendering::GpuRaysPtr gpuRays;

  /// \brief Connection to the Manager's scene change event.
  public: gz::common::ConnectionPtr sceneChangeConnection;

  /// \brief Event that is used to trigger callbacks when a new
  /// lidar frame is available
  public: gz::common::EventT<
          void(const float *_scan, unsigned int _width,
               unsigned int _height, unsigned int _channels,
               const std::string &_format)> lidarEvent;

  /// \brief Callback when new lidar frame is received
  public: void OnNewLidarFrame(const float *_scan, unsigned int _width,
               unsigned int _height, unsigned int _channels,
               const std::string &_format);

  /// \brief Connection to gpuRays new lidar frame event
  public: gz::common::ConnectionPtr lidarFrameConnection;

};

//////////////////////////////////////////////////
NavSatMultipathSensor::NavSatMultipathSensor()
  : dataPtr(std::make_unique<NavSatMultipathPrivate>())
{
}

//////////////////////////////////////////////////
NavSatMultipathSensor::~NavSatMultipathSensor()
{
  this->RemoveGpuRays(this->Scene());

  this->dataPtr->sceneChangeConnection.reset();

  if (this->laserBuffer)
  {
    delete [] this->laserBuffer;
    this->laserBuffer = nullptr;
  }
}

//////////////////////////////////////////////////
bool NavSatMultipathSensor::Init()
{
  return this->Sensor::Init();
}

//////////////////////////////////////////////////
bool NavSatMultipathSensor::Load(const sdf::Sensor &_sdf)
{
  if (!Sensor::Load(_sdf))
    return false;

  if (_sdf.Type() != sdf::SensorType::NAVSAT_MULTIPATH)
  {
    gzerr << "Attempting to a load an NAVSAT sensor, but received "
      << "a " << _sdf.TypeStr() << std::endl;
    return false;
  }
  if (_sdf.NavSatMultipathSensor() == nullptr)
  {
    gzerr << "Attempting to a load an NAVSAT sensor, but received "
      << "a null sensor." << std::endl;
    return false;
  }

  if (this->Topic().empty())
    this->SetTopic("/navsat_multipath");

  this->dataPtr->pub =
      this->dataPtr->node.Advertise<msgs::NavSat>(this->Topic());

  if (!this->dataPtr->pub)
  {
    gzerr << "Unable to create publisher on topic [" << this->Topic()
           << "]." << std::endl;
    return false;
  }

  // Load the noise parameters
  if (_sdf.NavSatMultipathSensor()->HorizontalPositionNoise().Type()
      != sdf::NoiseType::NONE)
  {
    this->dataPtr->noises[NAVSAT_HORIZONTAL_POSITION_NOISE] =
      NoiseFactory::NewNoiseModel(
        _sdf.NavSatMultipathSensor()->HorizontalPositionNoise());
  }
  if (_sdf.NavSatMultipathSensor()->VerticalPositionNoise().Type()
      != sdf::NoiseType::NONE)
  {
    this->dataPtr->noises[NAVSAT_VERTICAL_POSITION_NOISE] =
      NoiseFactory::NewNoiseModel(
        _sdf.NavSatMultipathSensor()->VerticalPositionNoise());
  }
  if (_sdf.NavSatMultipathSensor()->HorizontalVelocityNoise().Type()
      != sdf::NoiseType::NONE)
  {
    this->dataPtr->noises[NAVSAT_HORIZONTAL_VELOCITY_NOISE] =
      NoiseFactory::NewNoiseModel(
        _sdf.NavSatMultipathSensor()->HorizontalVelocityNoise());
  }
  if (_sdf.NavSatMultipathSensor()->VerticalVelocityNoise().Type()
      != sdf::NoiseType::NONE)
  {
    this->dataPtr->noises[NAVSAT_VERTICAL_VELOCITY_NOISE] =
      NoiseFactory::NewNoiseModel(
        _sdf.NavSatMultipathSensor()->VerticalVelocityNoise());
  }

  if (this->Scene())
  {
    this->CreateLidar();
  }
  this->dataPtr->sceneChangeConnection =
    RenderingEvents::ConnectSceneChangeCallback(
        std::bind(&NavSatMultipathSensor::SetScene, this, std::placeholders::_1));
  
  this->dataPtr->loaded = true;
  this->initialized = true;

  return true;
}

//////////////////////////////////////////////////
bool NavSatMultipathSensor::Load(sdf::ElementPtr _sdf)
{
  sdf::Sensor sdfSensor;
  sdfSensor.Load(_sdf);
  return this->Load(sdfSensor);
}

/////////////////////////////////////////////////
void NavSatMultipathSensor::SetScene(gz::rendering::ScenePtr _scene)
{
  std::lock_guard<std::mutex> lock(this->lidarMutex);
  // APIs make it possible for the scene pointer to change
  if (this->Scene() != _scene)
  {
    this->RemoveGpuRays(this->Scene());
    RenderingSensor::SetScene(_scene);

    if (this->initialized)
      this->CreateLidar();
  }
}

//////////////////////////////////////////////////
void NavSatMultipathSensor::RemoveGpuRays(
    gz::rendering::ScenePtr _scene)
{
  if (_scene)
  {
    _scene->DestroySensor(this->dataPtr->gpuRays);
  }
  this->dataPtr->gpuRays.reset();
  this->dataPtr->gpuRays = nullptr;
}


//////////////////////////////////////////////////
bool NavSatMultipathSensor::CreateLidar()
{
  this->dataPtr->gpuRays = this->Scene()->CreateGpuRays(
      this->Name());

  if (!this->dataPtr->gpuRays)
  {
    gzerr << "Unable to create gpu laser sensor\n";
    return false;
  }
  this->dataPtr->gpuRays->SetWorldPosition(this->Pose().Pos());
  this->dataPtr->gpuRays->SetWorldRotation(this->Pose().Rot());

  this->dataPtr->gpuRays->SetNearClipPlane(this->dataPtr->minRange);
  this->dataPtr->gpuRays->SetFarClipPlane(this->dataPtr->maxRange);

  // Mask ranges outside of min/max to +/- inf, as per REP 117
  this->dataPtr->gpuRays->SetClamp(false);

  this->dataPtr->gpuRays->SetAngleMin(0);
  this->dataPtr->gpuRays->SetAngleMax(gz::math::Angle::TwoPi.Radian());

  this->dataPtr->gpuRays->SetVerticalAngleMin(
      0);
  this->dataPtr->gpuRays->SetVerticalAngleMax(
      gz::math::Angle::HalfPi.Radian());

  this->dataPtr->gpuRays->SetRayCount(this->dataPtr->horizontalRayCount);
  this->dataPtr->gpuRays->SetVerticalRayCount(
      this->dataPtr->verticalRayCount);

  this->Scene()->RootVisual()->AddChild(
      this->dataPtr->gpuRays);

  this->dataPtr->lidarFrameConnection =
      this->dataPtr->gpuRays->ConnectNewGpuRaysFrame(
      std::bind(&NavSatMultipathSensor::OnNewLidarFrame, this,
      std::placeholders::_1, std::placeholders::_2, std::placeholders::_3,
      std::placeholders::_4, std::placeholders::_5));

  this->AddSensor(this->dataPtr->gpuRays);

  return true;
}

/////////////////////////////////////////////////
void NavSatMultipathSensor::OnNewLidarFrame(const float *_data,
    unsigned int _width, unsigned int _height, unsigned int _channels,
    const std::string &_format)
{
  std::lock_guard<std::mutex> lock(this->lidarMutex);

  unsigned int samples = _width * _height * _channels;
  unsigned int lidarBufferSize = samples * sizeof(float);

  if (!this->laserBuffer)
    this->laserBuffer = new float[samples];

  memcpy(this->laserBuffer, _data, lidarBufferSize);  
  if (this->dataPtr->lidarEvent.ConnectionCount() > 0)
  {
    this->dataPtr->lidarEvent(_data, _width, _height, _channels, _format);
  }
}

/////////////////////////////////////////////////
gz::common::ConnectionPtr NavSatMultipathSensor::ConnectNewLidarFrame(
          std::function<void(const float *_scan, unsigned int _width,
                  unsigned int _height, unsigned int _channels,
                  const std::string &/*_format*/)> _subscriber)
{
  return this->dataPtr->lidarEvent.Connect(_subscriber);
}

/////////////////////////////////////////////////
gz::rendering::GpuRaysPtr NavSatMultipathSensor::GpuRays() const
{
  return this->dataPtr->gpuRays;
}

//////////////////////////////////////////////////
bool NavSatMultipathSensor::Update(const std::chrono::steady_clock::duration &_now)
{
  GZ_PROFILE("NavSatMultipathSensor::Update");
  if (!this->dataPtr->loaded)
  {
    gzerr << "Not loaded, update ignored.\n";
    return false;
  }
  if (!this->initialized)
  {
    gzerr << "Not initialized, update ignored.\n";
    return false;
  }

  if (!this->dataPtr->gpuRays)
  {
    gzerr << "GpuRays doesn't exist.\n";
    return false;
  }

  this->Render();


  msgs::NavSat msg;
  *msg.mutable_header()->mutable_stamp() = msgs::Convert(_now);
  msg.set_frame_id(this->FrameId());

  // Apply noise
  auto iter = this->dataPtr->noises.find(NAVSAT_HORIZONTAL_POSITION_NOISE);
  if (iter != this->dataPtr->noises.end())
  {
    this->SetLatitude(GZ_DTOR(iter->second->Apply(this->Latitude().Degree())));
    this->SetLongitude(GZ_DTOR(iter->second->Apply(
        this->Longitude().Degree())));
  }
  iter = this->dataPtr->noises.find(NAVSAT_VERTICAL_POSITION_NOISE);
  if (iter != this->dataPtr->noises.end())
  {
    this->SetAltitude(iter->second->Apply(this->Altitude()));
  }
  iter = this->dataPtr->noises.find(NAVSAT_HORIZONTAL_VELOCITY_NOISE);
  if (iter != this->dataPtr->noises.end())
  {
    this->dataPtr->velocity.X(iter->second->Apply(this->dataPtr->velocity.X()));
    this->dataPtr->velocity.Y(iter->second->Apply(this->dataPtr->velocity.Y()));
  }
  iter = this->dataPtr->noises.find(NAVSAT_VERTICAL_VELOCITY_NOISE);
  if (iter != this->dataPtr->noises.end())
  {
    this->dataPtr->velocity.Z(iter->second->Apply(this->dataPtr->velocity.Z()));
  }

  // normalise so that it is within +/- 180
  this->dataPtr->latitude.Normalize();
  this->dataPtr->longitude.Normalize();
  
  msg.set_latitude_deg(this->dataPtr->latitude.Degree());
  msg.set_longitude_deg(this->dataPtr->longitude.Degree());
  msg.set_altitude(this->dataPtr->altitude);
  msg.set_velocity_east(this->dataPtr->velocity.X());
  msg.set_velocity_north(this->dataPtr->velocity.Y());
  msg.set_velocity_up(this->dataPtr->velocity.Z());

  // publish
  this->AddSequence(msg.mutable_header());
  this->dataPtr->pub.Publish(msg);

  return true;
}

//////////////////////////////////////////////////
void NavSatMultipathSensor::SetLatitude(const math::Angle &_latitude)
{
  this->dataPtr->latitude = _latitude;
}

//////////////////////////////////////////////////
const math::Angle &NavSatMultipathSensor::Latitude() const
{
  return this->dataPtr->latitude;
}

//////////////////////////////////////////////////
void NavSatMultipathSensor::SetAltitude(double _altitude)
{
  this->dataPtr->altitude = _altitude;
}

//////////////////////////////////////////////////
double NavSatMultipathSensor::Altitude() const
{
  return this->dataPtr->altitude;
}

//////////////////////////////////////////////////
void NavSatMultipathSensor::SetLongitude(const math::Angle &_longitude)
{
  this->dataPtr->longitude = _longitude;
}

//////////////////////////////////////////////////
const math::Angle &NavSatMultipathSensor::Longitude() const
{
  return this->dataPtr->longitude;
}

//////////////////////////////////////////////////
void NavSatMultipathSensor::SetVelocity(const math::Vector3d &_vel)
{
  this->dataPtr->velocity = _vel;
}

//////////////////////////////////////////////////
const math::Vector3d &NavSatMultipathSensor::Velocity() const
{
  return this->dataPtr->velocity;
}

//////////////////////////////////////////////////
void NavSatMultipathSensor::SetPosition(const math::Angle &_latitude,
    const math::Angle &_longitude, double _altitude)
{
  this->SetLatitude(_latitude);
  this->SetLongitude(_longitude);
  this->SetAltitude(_altitude);
}

//////////////////////////////////////////////////
bool NavSatMultipathSensor::HasConnections() const
{
  return Lidar::HasConnections()|| (this->dataPtr->pub && this->dataPtr->pub.HasConnections())
  || this->dataPtr->lidarEvent.ConnectionCount() > 0u;
}

//////////////////////////////////////////////////
void NavSatMultipathSensor::GetRangefromAzimuthElevation(float *_range,
   float azimuth, float elevation)
{
  GZ_PROFILE("NavSatMultipathSensor::GetRangefromAzimuthElevation");
  uint32_t width = this->dataPtr->horizontalRayCount;
  uint32_t height = this->dataPtr->verticalRayCount;
  unsigned int channels = 3;

  uint32_t i = (uint32_t)std::floor(azimuth/gz::math::Angle::TwoPi.Radian() * width);
  uint32_t j = (uint32_t)std::floor(elevation/gz::math::Angle::HalfPi.Radian() * height);
  auto index =  j * width * channels + i * channels;
  *_range = this->laserBuffer[index];
  *_range = gz::math::isnan(*_range) ? this->dataPtr->maxRange : *_range;
  
}