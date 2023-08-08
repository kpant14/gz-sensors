/*
 * Copyright (C) 2021 Open Source Robotics Foundation
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
*/
#ifndef GZ_SENSORS_NAVSAT_MULTIPATH_HH_
#define GZ_SENSORS_NAVSAT_MULTIPATH_HH_

#include <memory>

#include <gz/utils/SuppressWarning.hh>
#include <sdf/Sensor.hh>
#include <gz/rendering/GpuRays.hh>
#include "gz/sensors/config.hh"
#include "gz/sensors/navsat_multipath/Export.hh"
#include "gz/sensors/RenderingEvents.hh"
#include "gz/sensors/Lidar.hh" 



namespace gz
{
  namespace sensors
  {
    // Inline bracket to help doxygen filtering.
    inline namespace GZ_SENSORS_VERSION_NAMESPACE {
    //
    /// \brief forward declarations
    class NavSatMultipathPrivate;

    /// \brief NavSat Sensor Class
    ///
    /// A sensor that reports position and velocity readings over
    /// Gazebo Transport using spherical coordinates (latitude / longitude).
    ///
    /// By default, it publishes `gz::msgs::NavSat` messages on the
    /// `/.../navsat` topic.
    ///
    /// This sensor assumes the world is using the East-North-Up (ENU) frame.
    class GZ_SENSORS_NAVSAT_MULTIPATH_VISIBLE NavSatMultipathSensor : public Lidar
    {
      /// \brief Constructor
      public: NavSatMultipathSensor();

      /// \brief Destructor
      public: virtual ~NavSatMultipathSensor();

      /// \brief Load the sensor based on data from an sdf::Sensor object.
      /// \param[in] _sdf SDF Sensor parameters.
      /// \return true if loading was successful
      public: virtual bool Load(const sdf::Sensor &_sdf) override;

      /// \brief Load the sensor with SDF parameters.
      /// \param[in] _sdf SDF Sensor parameters.
      /// \return true if loading was successful
      public: virtual bool Load(sdf::ElementPtr _sdf) override;

      /// \brief Initialize values in the sensor
      /// \return True on success
      public: virtual bool Init() override;

      /// \brief Update the sensor and generate data
      /// \param[in] _now The current time
      /// \return true if the update was successfull
      public: virtual bool Update(
        const std::chrono::steady_clock::duration &_now) override;

      /// \brief Set the latitude of the NavSat
      /// \param[in] _latitude Latitude of NavSat
      public: void SetLatitude(const math::Angle &_latitude);

      /// \brief Get the latitude of the NavSat, wrapped between +/- 180
      /// degrees.
      /// \return Latitude angle.
      public: const math::Angle &Latitude() const;

      /// \brief Set the longitude of the NavSat
      /// \param[in] _longitude Longitude of NavSat
      public: void SetLongitude(const math::Angle &_longitude);

      /// \brief Get the longitude of the NavSat, wrapped between +/- 180
      /// degrees.
      /// \return Longitude angle.
      public: const math::Angle &Longitude() const;

      /// \brief Set the altitude of the NavSat
      /// \param[in] _altitude altitude of NavSat in meters
      public: void SetAltitude(double _altitude);

      /// \brief Get NavSat altitude above sea level
      /// \return Altitude in meters
      public: double Altitude() const;

      /// \brief Set the velocity of the NavSat in ENU world frame.
      /// \param[in] _vel NavSat in meters per second.
      public: void SetVelocity(const math::Vector3d &_vel);

      /// \brief Get the velocity of the NavSat sensor in the ENU world frame.
      /// \return Velocity in meters per second
      public: const math::Vector3d &Velocity() const;

      /// \brief Check if there are any subscribers
      /// \return True if there are subscribers, false otherwise
      public: virtual bool HasConnections() const override;

      /// \brief Easy short hand for setting the position of the sensor.
      /// \param[in] _latitude Latitude angle.
      /// \param[in] _longitude Longitude angle.
      /// \param[in] _altitude Altitude in meters; defaults to zero.
      public: void SetPosition(const math::Angle &_latitude,
          const math::Angle &_longitude, double _altitude = 0.0);


      /// \brief Create Lidar sensor
      public: virtual bool CreateLidar() override;

      /// \brief Makes possible to change sensor scene
      /// \param[in] _scene used with the sensor
      public: void SetScene(gz::rendering::ScenePtr _scene) override;

      /// \brief Remove sensor from scene
      /// \param[in] _scene used with the sensor
      public: void RemoveGpuRays(gz::rendering::ScenePtr _scene);

      /// \brief Get Gpu Rays object used in the sensor
      /// \return Pointer to gz::rendering::GpuRays
      public: gz::rendering::GpuRaysPtr GpuRays() const;

      /// \brief Connect function pointer to internal GpuRays callback
      /// \return gz::common::Connection pointer
      public: virtual gz::common::ConnectionPtr ConnectNewLidarFrame(
          std::function<void(const float *_scan, unsigned int _width,
                  unsigned int _heighti, unsigned int _channels,
                  const std::string &/*_format*/)> _subscriber) override;

      /// \brief Connect function pointer to internal GpuRays callback
      /// \return gz::common::Connection pointer
      private: void OnNewLidarFrame(const float *_scan, unsigned int _width,
                  unsigned int _heighti, unsigned int _channels,
                  const std::string &_format);

      GZ_UTILS_WARN_IGNORE__DLL_INTERFACE_MISSING
      /// \brief Data pointer for private data
      /// \internal
      private: std::unique_ptr<NavSatMultipathPrivate> dataPtr;
      GZ_UTILS_WARN_RESUME__DLL_INTERFACE_MISSING
    };
    }
  }
}

#endif



// /*
//  * Copyright (C) 2018 Open Source Robotics Foundation
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
// #ifndef GZ_SENSORS_NAVSAT_MULTIPATH_HH_
// #define GZ_SENSORS_NAVSAT_MULTIPATH_HH_

// #include <memory>
// #include <string>

// #include <sdf/sdf.hh>

// #include <gz/utils/SuppressWarning.hh>

// // TODO(louise) Remove these pragmas once gz-rendering is disabling the
// // warnings
// #ifdef _WIN32
// #pragma warning(push)
// #pragma warning(disable: 4251)
// #endif
// #include <gz/rendering/GpuRays.hh>
// #ifdef _WIN32
// #pragma warning(pop)
// #endif

// #include "gz/sensors/navsat_multipath/Export.hh"
// #include "gz/sensors/RenderingEvents.hh"
// #include "gz/sensors/Lidar.hh"

// namespace gz
// {
//   namespace sensors
//   {
//     // Inline bracket to help doxygen filtering.
//     inline namespace GZ_SENSORS_VERSION_NAMESPACE {
//     //
//     /// \brief forward declarations
//     class NavSatMultipathSensorPrivate;

//     /// \brief NavSatMultipath Sensor Class
//     ///
//     ///   This class creates laser scans using the GPU. It's measures the range
//     ///   from the origin of the center to points on the visual geometry in the
//     ///   scene.
//     ///
//     ///   It offers both a gz-transport interface and a direct C++ API
//     ///   to access the image data. The API works by setting a callback to be
//     ///   called with image data.
//     class GZ_SENSORS_NAVSAT_MULTIPATH_VISIBLE NavSatMultipathSensor : public Lidar
//     {
//       /// \brief constructor
//       public: NavSatMultipathSensor();

//       /// \brief destructor
//       public: virtual ~NavSatMultipathSensor();

//       /// \brief Force the sensor to generate data
//       /// \param[in] _now The current time
//       /// \return true if the update was successfull
//       public: virtual bool Update(
//         const std::chrono::steady_clock::duration &_now) override;

//       /// \brief Initialize values in the sensor
//       /// \return True on success
//       public: virtual bool Init() override;

//       /// \brief Load the sensor based on data from an sdf::Sensor object.
//       /// \param[in] _sdf SDF Sensor parameters.
//       /// \return true if loading was successful
//       public: virtual bool Load(const sdf::Sensor &_sdf) override;

//       /// \brief Load sensor sata from SDF
//       /// \param[in] _sdf SDF used
//       /// \return True on success
//       public: virtual bool Load(sdf::ElementPtr _sdf) override;

//       /// \brief Create Lidar sensor
//       public: virtual bool CreateLidar() override;

//       /// \brief Gets if sensor is horizontal
//       /// \return True if horizontal, false if not
//       public: bool IsHorizontal() const;

//       /// \brief Makes possible to change sensor scene
//       /// \param[in] _scene used with the sensor
//       public: void SetScene(gz::rendering::ScenePtr _scene) override;

//       /// \brief Remove sensor from scene
//       /// \param[in] _scene used with the sensor
//       public: void RemoveGpuRays(gz::rendering::ScenePtr _scene);

//       /// \brief Get Gpu Rays object used in the sensor
//       /// \return Pointer to gz::rendering::GpuRays
//       public: gz::rendering::GpuRaysPtr GpuRays() const;

//       /// \brief Return the ratio of horizontal ray count to vertical ray
//       /// count.
//       ///
//       /// A ray count is the number of simulated rays. Whereas a range count
//       /// is the total number of data points returned. When range count
//       /// != ray count, then values are interpolated between rays.
//       public: double RayCountRatio() const;

//       /// \brief Get the horizontal field of view of the laser sensor.
//       /// \return The horizontal field of view of the laser sensor.
//       public: gz::math::Angle HFOV() const;

//       /// \brief Get the vertical field-of-view.
//       /// \return Vertical field of view.
//       public: gz::math::Angle VFOV() const;

//       /// \brief Check if there are any subscribers
//       /// \return True if there are subscribers, false otherwise
//       public: virtual bool HasConnections() const override;

//       /// \brief Connect function pointer to internal GpuRays callback
//       /// \return gz::common::Connection pointer
//       public: virtual gz::common::ConnectionPtr ConnectNewLidarFrame(
//           std::function<void(const float *_scan, unsigned int _width,
//                   unsigned int _heighti, unsigned int _channels,
//                   const std::string &/*_format*/)> _subscriber) override;

//       /// \brief Connect function pointer to internal GpuRays callback
//       /// \return gz::common::Connection pointer
//       private: void OnNewLidarFrame(const float *_scan, unsigned int _width,
//                   unsigned int _heighti, unsigned int _channels,
//                   const std::string &_format);

//       GZ_UTILS_WARN_IGNORE__DLL_INTERFACE_MISSING
//       /// \brief Data pointer for private data
//       /// \internal
//       private: std::unique_ptr<NavSatMultipathSensorPrivate> dataPtr;
//       GZ_UTILS_WARN_RESUME__DLL_INTERFACE_MISSING
//     };
//     }
//   }
// }

// #endif

