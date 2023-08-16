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

#include <predict/predict.h>

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


  // For Satellite Library
  public: int numSat;
  // TlE parameters to store the satellite's orbital parameters
  public: const char **tleLines;

  // Satellite orbital parameters to predict satellite's position
  public: predict_orbital_elements_t **satOrbitElements;

  // Satellite position information at a given time
  public: struct predict_position *satPosition;

  public: double worldOriginLat, worldOriginLon, worldOriginAlt; 

  public: gz::sensors::NavSatConverter navsatConverter;

  // Time information to get the satellite position
  public: struct tm *timeinfo;

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
  this->dataPtr->navsatConverter.initialiseReference(this->dataPtr->worldOriginLat, 
        this->dataPtr->worldOriginLon, this->dataPtr->worldOriginAlt);
  
  this->ParseSatelliteTLE();

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


  std::vector<double> satTrueRange;
  std::vector<std::vector<double>> satECEF;
  std::vector<double> elevation;
  std::vector<double> azimuth; 
  std::vector<double> rayElevation;
  std::vector<double> rayAzimuth;
  std::vector<double> recLLA;
  
  recLLA.push_back(this->dataPtr->latitude.Degree());
  recLLA.push_back(this->dataPtr->longitude.Degree());
  recLLA.push_back(this->dataPtr->altitude);
  
    // Constructing a time structure for getting satellites position
  this->dataPtr->timeinfo = (struct tm*)calloc(1, sizeof(struct tm));  
  this->dataPtr->timeinfo->tm_year = 2021 - 1900;
  this->dataPtr->timeinfo->tm_mon = 5 - 1;
  this->dataPtr->timeinfo->tm_mday = 15;
  this->dataPtr->timeinfo->tm_hour = 1;
  this->dataPtr->timeinfo->tm_min = 55;
  this->dataPtr->timeinfo->tm_sec = 0;
  this->dataPtr->timeinfo->tm_isdst = 0;
  // Collect positive elevation satellites ECEF, true ranges and angles 
  GetSatellitesInfo(this->dataPtr->timeinfo, recLLA, satECEF, satTrueRange, azimuth, elevation);
  int activeNumSat = satTrueRange.size();
  //gzerr << "num sat: "<< activeNumSat<< std::endl;
  // float range;
  // GetRangefromAzimuthElevation(&range, 0, 0 );
  // gzerr <<"O deg:" << range << std::endl;
  // GetRangefromAzimuthElevation(&range, M_PI_2, 0 );
  // gzerr <<"90 deg:"<< range << std::endl;
  // GetRangefromAzimuthElevation(&range, M_PI, 0 );
  // gzerr <<"180 deg:"<< range << std::endl;
  // GetRangefromAzimuthElevation(&range, 1.5 * M_PI, 0 );
  // gzerr << "270 deg:" <<range << std::endl;


  //Ray_ranges stores the ranges of both satellite and reflected rays.
  std::vector<double> rayRanges; 
  for ( int i = 0; i < activeNumSat; i++ )
  { 
    float range, rangeReflected;
    // Setting the azimuth and elevation angles for the satellite ray and the reflected ray for each satellite.
    rayAzimuth.push_back(azimuth[i]);
    rayAzimuth.push_back(azimuth[i]+ M_PI); // assume reflection is 180 deg azimuth from the direct ray
    rayElevation.push_back(elevation[i]);
    rayElevation.push_back(elevation[i]); // assume reflection has the same elevation as the direct ray
    //gzerr << "azimuth: " << azimuth[i]*180/M_PI << "elevation: " << elevation[i]*180/M_PI<<std::endl;
    GetRangefromAzimuthElevation(&range, azimuth[i], elevation[i] );
    GetRangefromAzimuthElevation(&rangeReflected, azimuth[i] + M_PI, elevation[i]);
    rayRanges.push_back(range);
    rayRanges.push_back(rangeReflected);
  }

  int numSatBlocked = 0;
  std::vector<double> visibleSatRangeMeas;
  std::vector<std::vector<double>> visibleSatECEF;
  
  // Iterate for all the visible satellites (positive elevation angles)
  for (int i = 0; i < activeNumSat; i++)
  {
    //gzerr << rayRanges[i]<<" "<< rayRanges[i+1] << std::endl; 
    if (rayElevation[2*i] > (15*M_PI)/180)
    {
      // ray is obstructed
      if (rayRanges[2*i] < this->dataPtr->maxRange) 
      {
        // check range of the mirror ray corresponding to the satellite ray 
        double mirRayRange = rayRanges[2*i+1];

        // check if mirror ray also is obstructed, providing a reflection
        if (mirRayRange < this->dataPtr->maxRange)
        {
          double rangeOffset =  mirRayRange * ( 1 + sin(M_PI/2.0 - 2*rayElevation[2*i]));
          // RCLCPP_INFO(ros_node_->get_logger(), "range:%f %f %f ", range_offset,mir_ray_range_, ray_elevation_[2*i] );
          // Ignore the offset if the multipath range is too high
          if (rangeOffset < 50)
          {
            //RCLCPP_INFO(ros_node_->get_logger(), "offset:%d %f", i, range_offset);
            visibleSatRangeMeas.push_back(satTrueRange[i] + rangeOffset);
            visibleSatECEF.push_back(satECEF[i]);
          }
          else
          {
            // The multipath distance is large, signal intensity will be degraded significantly, so ignored.
            numSatBlocked++;
          }
        } 
        else 
        {
          // otherwise no line of sight to any satellites and no reflections, no reading
          numSatBlocked++;
        }
      } 
      else 
      {
        // unobstructed sat reading
        visibleSatRangeMeas.push_back(satTrueRange[i]);
        visibleSatECEF.push_back(satECEF[i]);
      }
    }
    else
    {
      // low elevation sat reading
      numSatBlocked++;
    }
  }
  if ((activeNumSat - numSatBlocked) > 4)
  {
    // Found a FIX 
    
    // Calculate the reciever's position using the satellite's predicted coordinates and the pseudo-ranges.
    Eigen::Vector3d recECEF(0,0,0);  
    std::vector<double> dop;
    double latitude=0, longitude=0, altitude=0;

    GetLeastSquaresEstimate(visibleSatRangeMeas,  visibleSatECEF, recECEF);
    CalculateDOP(visibleSatECEF, recECEF, dop);
    
    this->dataPtr->navsatConverter.ecef2Geodetic(recECEF(0), recECEF(1),recECEF(2), &latitude,
                      &longitude, &altitude);
    
    // //Copy DOP values to the message.
    // std::copy(dop.begin(),
    //         dop.end(),
    //         gnss_multipath_fix_msg.dop.begin());
    msg.set_latitude_deg(latitude);
    msg.set_longitude_deg(longitude);
    msg.set_altitude(altitude);
    msg.set_velocity_east(this->dataPtr->velocity.X());
    msg.set_velocity_north(this->dataPtr->velocity.Y());
    msg.set_velocity_up(this->dataPtr->velocity.Z());
  }
  else 
  {
    // No FIX
    msg.set_latitude_deg(NAN);
    msg.set_longitude_deg(NAN);
    msg.set_altitude(NAN);
    msg.set_velocity_east(NAN);
    msg.set_velocity_north(NAN);
    msg.set_velocity_up(NAN);
  }
  // msg.set_latitude_deg(this->dataPtr->latitude.Degree());
  // msg.set_longitude_deg(this->dataPtr->longitude.Degree());
  // msg.set_altitude(this->dataPtr->altitude);
  // msg.set_velocity_east(this->dataPtr->velocity.X());
  // msg.set_velocity_north(this->dataPtr->velocity.Y());
  // msg.set_velocity_up(this->dataPtr->velocity.Z());
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


bool NavSatMultipathSensor::GetLeastSquaresEstimate(std::vector<double> _meas,
                 std::vector<std::vector<double>> _sat_ecef, Eigen::Vector3d &_rec_ecef)
{
  GZ_PROFILE("NavSatMultipathSensor::GetLeastSquaresEstimate");
  const int nsat = _meas.size();
  Eigen::MatrixXd A, b;
  A.resize(nsat, 4);
  b.resize(nsat, 1);
  Eigen::Matrix<double, 4, 1> dx;
  Eigen::ColPivHouseholderQR<Eigen::MatrixXd> solver;
  double cdt = 0;
  int iter = 0;
  do
  {
    iter++;
    for (int i = 0 ; i < nsat; i++)
    {
      Eigen::Vector3d sat_pos(_sat_ecef[i][0], _sat_ecef[i][1], _sat_ecef[i][2]);
      double dist = (_rec_ecef - sat_pos).norm();
      A.block<1,3>(i,0) = (_rec_ecef - sat_pos).normalized();
      b(i) = _meas[i] - (dist + cdt);
      A(i,3) = -1;
      
    }
    solver.compute(A);
    dx = solver.solve(b);
    _rec_ecef += dx.topRows<3>();
  } while (dx.norm() > 1e-6 && iter < 10);
  return iter < 10;
}

void NavSatMultipathSensor::CalculateDOP(std::vector<std::vector<double>> _sat_ecef, 
                    Eigen::Vector3d _rec_ecef, std::vector<double> &_dop)
{
  GZ_PROFILE("NavSatMultipathSensor::CalculateDOP");
  const int nsat = _sat_ecef.size();
 
  Eigen::MatrixXd A, Q, Q_local;
  A.resize(nsat, 4);
  for (int i = 0 ; i < nsat; i++)
  {
    Eigen::Vector3d sat_pos(_sat_ecef[i][0], _sat_ecef[i][1], _sat_ecef[i][2]);
    A.block<1,3>(i,0) = (_rec_ecef - sat_pos).normalized();
    A(i,3) = -1;
  }
  Q = (A.transpose()*A).inverse();
  //GDOP
  _dop.push_back(std::sqrt(Q.trace()));
  //PDOP
  _dop.push_back(std::sqrt((Q.block<3,3>(0,0)).trace()));
	//TDOP
  _dop.push_back(std::sqrt(Q(3,3)));
  double latitude = 0.0, longitude=0.0, altitude=0.0;
  this->dataPtr->navsatConverter.ecef2Geodetic(_rec_ecef(0), _rec_ecef(1),_rec_ecef(2), &latitude,
                      &longitude, &altitude);
	double phi = latitude; 			
	double lambda = longitude;	

  double sl = sin(lambda);
  double cl = cos(lambda);
  double sp = sin(phi);
  double cp = cos(phi);
  
  Eigen::MatrixXd Rot_matrix(4, 4);
  Rot_matrix(0, 0) = -cl*sp;
  Rot_matrix(0, 1) = -sl*sp;
  Rot_matrix(0, 2) = cp;
  Rot_matrix(0, 3) = 0;

  Rot_matrix(1, 0) = -sl;
  Rot_matrix(1, 1) = cl;
  Rot_matrix(1, 2) = 0;
  Rot_matrix(1, 3) = 0;

  Rot_matrix(2, 0) = cl*cp;
  Rot_matrix(2, 1) = cp*sl;
  Rot_matrix(2, 2) = sp;
  Rot_matrix(2, 3) = 0;

  Rot_matrix(3, 0) = 0;
  Rot_matrix(3, 1) = 0;
  Rot_matrix(3, 2) = 0;
  Rot_matrix(3, 3) = 1;

	// calculate the local cofactor matrix
	Q_local = Rot_matrix*Q*Rot_matrix.transpose();
  // calculate 'HDOP' and 'VDOP' 
	// HDOP
  _dop.push_back(std::sqrt(Q_local(0,0)*Q_local(1,1)));
	// VDOP
  _dop.push_back(std::sqrt(Q_local(2,2)));
}

time_t NavSatMultipathSensor::MakeTimeUTC(const struct tm* timeinfo_utc)
{
	time_t curr_time = time(NULL);
	int timezone_diff = 0; //deviation of the current timezone from UTC in seconds

	//get UTC time, interpret resulting tm as a localtime
	struct tm timeinfo_gmt;
	gmtime_r(&curr_time, &timeinfo_gmt);
	time_t time_gmt = mktime(&timeinfo_gmt);

	//get localtime, interpret resulting tm as localtime
	struct tm timeinfo_local;
	localtime_r(&curr_time, &timeinfo_local);
	time_t time_local = mktime(&timeinfo_local);

	//find the time difference between the two interpretations
	timezone_diff += difftime(time_local, time_gmt);

	//hack for preventing mktime from assuming localtime: add timezone difference to the input struct.
	struct tm ret_timeinfo;
	ret_timeinfo.tm_sec = timeinfo_utc->tm_sec + timezone_diff;
	ret_timeinfo.tm_min = timeinfo_utc->tm_min;
	ret_timeinfo.tm_hour = timeinfo_utc->tm_hour;
	ret_timeinfo.tm_mday = timeinfo_utc->tm_mday;
	ret_timeinfo.tm_mon = timeinfo_utc->tm_mon;
	ret_timeinfo.tm_year = timeinfo_utc->tm_year;
	ret_timeinfo.tm_isdst = timeinfo_utc->tm_isdst;
	return mktime(&ret_timeinfo);
}

void NavSatMultipathSensor::GetSatellitesInfo(struct tm *_timeinfo, std::vector<double> _rec_lla,
      std::vector<std::vector<double>> &_sat_ecef, std::vector<double> &_true_range, std::vector<double> &_azimuth,
      std::vector<double> &_elevation)
{
  GZ_PROFILE("NavSatMultipathSensor::GetSatellitesInfo");
  // Predict the julian time using the current utc time
  predict_julian_date_t curr_time = predict_to_julian(MakeTimeUTC(_timeinfo));
  std::vector<double> ecef = {0,0,0};
  for ( int i = 0; i < this->dataPtr->numSat; i++ )
  {
    // Predict satellite's position based on the current time.
    predict_orbit(this->dataPtr->satOrbitElements[i], &this->dataPtr->satPosition[i], curr_time);
    this->dataPtr->navsatConverter.geodetic2Ecef(this->dataPtr->satPosition[i].latitude*180.0/M_PI, this->dataPtr->satPosition[i].longitude*180.0/M_PI , 
                  this->dataPtr->satPosition[i].altitude*1000 , &ecef[0], &ecef[1], &ecef[2]);
    predict_observer_t *obs = predict_create_observer("obs", _rec_lla[0]*M_PI/180.0, _rec_lla[1]*M_PI/180.0, _rec_lla[2]);
    struct predict_observation reciever_obs;
    predict_observe_orbit(obs, &this->dataPtr->satPosition[i], &reciever_obs);
    if (reciever_obs.elevation > 0)
    {
      _sat_ecef.push_back(ecef);
      _true_range.push_back(reciever_obs.range*1000); //From Km to m  
      _azimuth.push_back(reciever_obs.azimuth);
      _elevation.push_back(reciever_obs.elevation);
    }
    
  }
}

void NavSatMultipathSensor::ParseSatelliteTLE()
{
  this->dataPtr->numSat = 31;
  // Initialization of data structures for satellite prediction 
  this->dataPtr->tleLines = (const char**) calloc(2*this->dataPtr->numSat, sizeof(const char*));
  this->dataPtr->satOrbitElements = (predict_orbital_elements_t**) calloc(this->dataPtr->numSat, sizeof(predict_orbital_elements_t*));
  this->dataPtr->satPosition = (struct predict_position*) calloc(this->dataPtr->numSat, sizeof(struct predict_position));
  for ( int i = 0; i < 2*this->dataPtr->numSat; i++ )
  {
      this->dataPtr->tleLines[i] = (const char*) calloc(100, sizeof(const char));
  }
  for ( int i = 0; i < this->dataPtr->numSat; i++ )
  {
      this->dataPtr->satOrbitElements[i] = (predict_orbital_elements_t*) calloc(1, sizeof( predict_orbital_elements_t));
  }
  
  this->dataPtr->tleLines[0] = "1 24876U 97035A   23033.20764968 -.00000038  00000+0  00000+0 0  9994";
  this->dataPtr->tleLines[1] = "2 24876  55.5459 146.7067 0065794  52.6664 307.9046  2.00564086187267";
 
  this->dataPtr->tleLines[2] = "1 26360U 00025A   23033.06431156 -.00000048  00000+0  00000+0 0  9992";
  this->dataPtr->tleLines[3] = "2 26360  54.2496  69.3979 0045611 194.8807 151.7827  2.00555919166578";

  this->dataPtr->tleLines[4] = "1 27663U 03005A   23033.62997381  .00000047  00000+0  00000+0 0  9997";
  this->dataPtr->tleLines[5] = "2 27663  55.3642 263.9131 0131905  42.8180 326.5456  2.00551168146626";

  this->dataPtr->tleLines[6] = "1 27704U 03010A   23033.55723828 -.00000091  00000+0  00000+0 0  9994";
  this->dataPtr->tleLines[7] = "2 27704  55.0899  13.9998 0249685 312.4590 223.4527  2.00565951145429";

  this->dataPtr->tleLines[8] = "1 28190U 04009A   23034.15509573 -.00000085  00000+0  00000+0 0  9992";
  this->dataPtr->tleLines[9] = "2 28190  55.8516 324.9248 0088848 128.9769  57.1221  2.00572247138304";

  this->dataPtr->tleLines[10] = "1 28474U 04045A   23033.60153542 -.00000090  00000+0  00000+0 0  9993";
  this->dataPtr->tleLines[11] = "2 28474  55.4150  14.2264 0202589 283.4591  74.0418  2.00562393133746";

  this->dataPtr->tleLines[12] = "1 28874U 05038A   23033.61665581 -.00000084  00000+0  00000+0 0  9995";
  this->dataPtr->tleLines[13] = "2 28874  55.9094 322.3891 0139570 278.1790 267.2996  2.00560021127153";

  this->dataPtr->tleLines[14] = "1 29486U 06042A   23033.78720829  .00000046  00000+0  00000+0 0  9993";
  this->dataPtr->tleLines[15] = "2 29486  54.6970 200.1224 0106835  27.4847 197.4334  2.00565983119862";

  this->dataPtr->tleLines[16] = "1 29601U 06052A   23032.92873802  .00000048  00000+0  00000+0 0  9990";
  this->dataPtr->tleLines[17] = "2 29601  55.3786 262.8889 0086684  75.9810 285.0549  2.00564342118716";
    
  this->dataPtr->tleLines[18] = "1 32260U 07047A   23033.23014611 -.00000051  00000+0  00000+0 0  9995";
  this->dataPtr->tleLines[19] = "2 32260  53.3839 131.0174 0145380  67.1246 294.4003  2.00563380112126";

  this->dataPtr->tleLines[20] = "1 32384U 07062A   23033.85587446 -.00000084  00000+0  00000+0 0  9994";
  this->dataPtr->tleLines[21] = "2 32384  56.0351 323.1715 0020102 142.8969  77.6124  2.00574229110895";

  this->dataPtr->tleLines[22] = "1 32711U 08012A   23032.64592938  .00000056  00000+0  00000+0 0  9990";
  this->dataPtr->tleLines[23] = "2 32711  54.4610 199.0507 0168701 232.8355 125.5954  2.00572261109059";

  this->dataPtr->tleLines[24] = "1 35752U 09043A   23034.15624287 -.00000035  00000+0  00000+0 0  9992";
  this->dataPtr->tleLines[25] = "2 35752  55.2549  76.2529 0055618  63.8864 319.4178  2.00571659 98673";

  this->dataPtr->tleLines[26] = "1 36585U 10022A   23033.21070384  .00000056  00000+0  00000+0 0  9997";
  this->dataPtr->tleLines[27] = "2 36585  54.6695 258.2376 0109114  58.2673 124.2846  2.00550759 92904";
  
  this->dataPtr->tleLines[28] = "1 37753U 11036A   23033.10118579 -.00000081  00000+0  00000+0 0  9997";
  this->dataPtr->tleLines[29] = "2 37753  56.6933  19.7748 0121088  52.9316 129.8426  2.00566862 84583";

  this->dataPtr->tleLines[30] = "1 38833U 12053A   23033.33039722  .00000038  00000+0  00000+0 0  9995";
  this->dataPtr->tleLines[31] = "2 38833  53.5190 193.9407 0135373  49.9651 311.1852  2.00564336 74752";

  this->dataPtr->tleLines[32] = "1 39166U 13023A   23033.18918996 -.00000082  00000+0  00000+0 0  9995";
  this->dataPtr->tleLines[33] = "2 39166  55.5293 318.6885 0111059  38.9232 321.8895  2.00565200 71193";

  this->dataPtr->tleLines[34] = "1 39533U 14008A   23033.19159522  .00000049  00000+0  00000+0 0  9992";
  this->dataPtr->tleLines[35] = "2 39533  53.6091 199.4781 0062019 209.5871 150.0417  2.00566418 65548";

  this->dataPtr->tleLines[36] = "1 39741U 14026A   23033.96686822 -.00000082  00000+0  00000+0 0  9994";
  this->dataPtr->tleLines[37] = "2 39741  56.6508  19.2656 0030744 308.2184  51.5125  2.00572503 63854";

  this->dataPtr->tleLines[38] = "1 40105U 14045A   23032.56334991 -.00000048  00000+0  00000+0 0  9995";
  this->dataPtr->tleLines[39] = "2 40105  54.7481 137.6538 0023243 114.6825 245.5467  2.00562646 61376";

  this->dataPtr->tleLines[40] = "1 40294U 14068A   23032.43441650 -.00000049  00000+0  00000+0 0  9997";
  this->dataPtr->tleLines[41] = "2 40294  55.9964  78.9035 0042465  56.8096 303.6631  2.00557360 60507";

  this->dataPtr->tleLines[42] = "1 40534U 15013A   23033.06804744  .00000058  00000+0  00000+0 0  9991";
  this->dataPtr->tleLines[43] = "2 40534  53.5692 255.1758 0077227  23.8247 336.6011  2.00567871 57134";

  this->dataPtr->tleLines[44] = "1 40730U 15033A   23033.72582779 -.00000076  00000+0  00000+0 0  9996";
  this->dataPtr->tleLines[45] = "2 40730  55.0217 317.3848 0082059  10.2474 349.9421  2.00566329 55333";

  this->dataPtr->tleLines[46] = "1 41019U 15062A   23033.28107185 -.00000041  00000+0  00000+0 0  9996";
  this->dataPtr->tleLines[47] = "2 41019  55.9834  78.7168 0085778 220.7636 138.6587  2.00562556 53141";

  this->dataPtr->tleLines[48] = "1 41328U 16007A   23033.36788295 -.00000045  00000+0  00000+0 0  9999";
  this->dataPtr->tleLines[49] = "2 41328  54.9593 138.3485 0067345 231.5397 127.8428  2.00559919 51156";

  this->dataPtr->tleLines[50] = "1 43873U 18109A   23032.57137684 -.00000044  00000+0  00000+0 0  9992";
  this->dataPtr->tleLines[51] = "2 43873  55.1375 140.8489 0023229 190.4825 204.0790  2.00558671 30373";

  this->dataPtr->tleLines[52] = "1 44506U 19056A   23033.40051584 -.00000083  00000+0  00000+0 0  9998";
  this->dataPtr->tleLines[53] = "2 44506  55.7692  19.9151 0029851 186.7930 347.2623  2.00561526 25382";

  this->dataPtr->tleLines[54] = "1 45854U 20041A   23034.26264002 -.00000033  00000+0  00000+0 0  9992";
  this->dataPtr->tleLines[55] = "2 45854  55.7045  77.1516 0029948 185.9714 192.1492  2.00567044 19363";

  this->dataPtr->tleLines[56] = "1 46826U 20078A   23032.06127720  .00000047  00000+0  00000+0 0  9997";
  this->dataPtr->tleLines[57] = "2 46826  54.4254 260.8119 0026417 188.1073  16.6882  2.00564630 16797";

  this->dataPtr->tleLines[58] = "1 48859U 21054A   23033.87596315 -.00000082  00000+0  00000+0 0  9990";
  this->dataPtr->tleLines[59] = "2 48859  55.2812  21.7836 0009051 219.9340  39.0694  2.00560997 12067";

  this->dataPtr->tleLines[60] = "1 55268U 23009A   23033.46134089  .00000045  00000+0  00000+0 0  9993";
  this->dataPtr->tleLines[61] = "2 55268  55.1007 196.8026 0008185  97.8399 262.2308  2.00570647   571";

  
  for ( int i = 0; i < this->dataPtr->numSat; i++ )
  {
      // Create orbit object
      this->dataPtr->satOrbitElements[i] = predict_parse_tle(this->dataPtr->tleLines[2*i], this->dataPtr->tleLines[2*i+1]);
  }
}
