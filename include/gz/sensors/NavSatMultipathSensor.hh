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
#include <eigen3/Eigen/Dense>



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

    // Geodetic system parameters
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

      private: void GetRangefromAzimuthElevation(float *_range,
          float azimuth, float elevation);


      private: void SetRayAngles(std::vector<double> _azimuth, std::vector<double> _elevation); 
      private: bool GetLeastSquaresEstimate(std::vector<double> _meas, std::vector<std::vector<double>> _sat_ecef, Eigen::Vector3d & _rec_ecef);
      private: void CalculateDOP(std::vector<std::vector<double>> _sat_ecef, 
                        Eigen::Vector3d _rec_ecef, std::vector<double> &_dop);
      private: void ParseSatelliteTLE();
      private: void GetSatellitesInfo(struct tm *_timeinfo, std::vector<double> _rec_lla,
          std::vector<std::vector<double>> &_sat_ecef, std::vector<double> &_true_range, std::vector<double> &_azimuth,
          std::vector<double> &_elevation);
      private: time_t MakeTimeUTC(const struct tm* timeinfo_utc);    
      
      GZ_UTILS_WARN_IGNORE__DLL_INTERFACE_MISSING
      /// \brief Data pointer for private data
      /// \internal
      private: std::unique_ptr<NavSatMultipathPrivate> dataPtr;
      GZ_UTILS_WARN_RESUME__DLL_INTERFACE_MISSING
    };

      // Geodetic system parameters
    static double kSemimajorAxis = 6378137;
    static double kSemiminorAxis = 6356752.3142;
    static double kFirstEccentricitySquared = 6.69437999014 * 0.001;
    static double kSecondEccentricitySquared = 6.73949674228 * 0.001;
    static double kFlattening = 1 / 298.257223563;

    class NavSatConverter
    {
    public:

      NavSatConverter()
      {
        haveReference_ = false;
      }

      ~NavSatConverter()
      {
      }

      // Default copy constructor and assignment operator are OK.

      bool isInitialised()
      {
        return haveReference_;
      }

      void getReference(double* latitude, double* longitude, double* altitude)
      {
        *latitude = initial_latitude_;
        *longitude = initial_longitude_;
        *altitude = initial_altitude_;
      }

      void initialiseReference(const double latitude, const double longitude, const double altitude)
      {
        // Save NED origin
        initial_latitude_ = deg2Rad(latitude);
        initial_longitude_ = deg2Rad(longitude);
        initial_altitude_ = altitude;

        // Compute ECEF of NED origin
        geodetic2Ecef(latitude, longitude, altitude, &initial_ecef_x_, &initial_ecef_y_, &initial_ecef_z_);

        // Compute ECEF to NED and NED to ECEF matrices
        double phiP = atan2(initial_ecef_z_, sqrt(pow(initial_ecef_x_, 2) + pow(initial_ecef_y_, 2)));

        ecef_to_ned_matrix_ = nRe(phiP, initial_longitude_);
        ned_to_ecef_matrix_ = nRe(initial_latitude_, initial_longitude_).transpose();

        haveReference_ = true;
      }

      void geodetic2Ecef(const double latitude, const double longitude, const double altitude, double* x,
                        double* y, double* z)
      {
        // Convert geodetic coordinates to ECEF.
        // http://code.google.com/p/pysatel/source/browse/trunk/coord.py?r=22
        double lat_rad = deg2Rad(latitude);
        double lon_rad = deg2Rad(longitude);
        double xi = sqrt(1 - kFirstEccentricitySquared * sin(lat_rad) * sin(lat_rad));
        *x = (kSemimajorAxis / xi + altitude) * cos(lat_rad) * cos(lon_rad);
        *y = (kSemimajorAxis / xi + altitude) * cos(lat_rad) * sin(lon_rad);
        *z = (kSemimajorAxis / xi * (1 - kFirstEccentricitySquared) + altitude) * sin(lat_rad);
      }

      void ecef2Geodetic(const double x, const double y, const double z, double* latitude,
                        double* longitude, double* altitude)
      {
        // Convert ECEF coordinates to geodetic coordinates.
        // J. Zhu, "Conversion of Earth-centered Earth-fixed coordinates
        // to geodetic coordinates," IEEE Transactions on Aerospace and
        // Electronic Systems, vol. 30, pp. 957-961, 1994.

        double r = sqrt(x * x + y * y);
        double Esq = kSemimajorAxis * kSemimajorAxis - kSemiminorAxis * kSemiminorAxis;
        double F = 54 * kSemiminorAxis * kSemiminorAxis * z * z;
        double G = r * r + (1 - kFirstEccentricitySquared) * z * z - kFirstEccentricitySquared * Esq;
        double C = (kFirstEccentricitySquared * kFirstEccentricitySquared * F * r * r) / pow(G, 3);
        double S = cbrt(1 + C + sqrt(C * C + 2 * C));
        double P = F / (3 * pow((S + 1 / S + 1), 2) * G * G);
        double Q = sqrt(1 + 2 * kFirstEccentricitySquared * kFirstEccentricitySquared * P);
        double r_0 = -(P * kFirstEccentricitySquared * r) / (1 + Q)
            + sqrt(
                0.5 * kSemimajorAxis * kSemimajorAxis * (1 + 1.0 / Q)
                    - P * (1 - kFirstEccentricitySquared) * z * z / (Q * (1 + Q)) - 0.5 * P * r * r);
        double U = sqrt(pow((r - kFirstEccentricitySquared * r_0), 2) + z * z);
        double V = sqrt(
            pow((r - kFirstEccentricitySquared * r_0), 2) + (1 - kFirstEccentricitySquared) * z * z);
        double Z_0 = kSemiminorAxis * kSemiminorAxis * z / (kSemimajorAxis * V);
        *altitude = U * (1 - kSemiminorAxis * kSemiminorAxis / (kSemimajorAxis * V));
        *latitude = rad2Deg(atan((z + kSecondEccentricitySquared * Z_0) / r));
        *longitude = rad2Deg(atan2(y, x));
      }

      void ecef2Ned(const double x, const double y, const double z, double* north, double* east,
                    double* down)
      {
        // Converts ECEF coordinate position into local-tangent-plane NED.
        // Coordinates relative to given ECEF coordinate frame.

        Eigen::Vector3d vect, ret;
        vect(0) = x - initial_ecef_x_;
        vect(1) = y - initial_ecef_y_;
        vect(2) = z - initial_ecef_z_;
        ret = ecef_to_ned_matrix_ * vect;
        *north = ret(0);
        *east = ret(1);
        *down = -ret(2);
      }

      void ned2Ecef(const double north, const double east, const double down, double* x, double* y,
                    double* z)
      {
        // NED (north/east/down) to ECEF coordinates
        Eigen::Vector3d ned, ret;
        ned(0) = north;
        ned(1) = east;
        ned(2) = -down;
        ret = ned_to_ecef_matrix_ * ned;
        *x = ret(0) + initial_ecef_x_;
        *y = ret(1) + initial_ecef_y_;
        *z = ret(2) + initial_ecef_z_;
      }

      void geodetic2Ned(const double latitude, const double longitude, const double altitude,
                        double* north, double* east, double* down)
      {
        // Geodetic position to local NED frame
        double x, y, z;
        geodetic2Ecef(latitude, longitude, altitude, &x, &y, &z);
        ecef2Ned(x, y, z, north, east, down);
      }

      void ned2Geodetic(const double north, const double east, const double down, double* latitude,
                        double* longitude, double* altitude)
      {
        // Local NED position to geodetic coordinates
        double x, y, z;
        ned2Ecef(north, east, down, &x, &y, &z);
        ecef2Geodetic(x, y, z, latitude, longitude, altitude);
      }

      void geodetic2Enu(const double latitude, const double longitude, const double altitude,
                        double* east, double* north, double* up)
      {
        // Geodetic position to local ENU frame
        double x, y, z;
        geodetic2Ecef(latitude, longitude, altitude, &x, &y, &z);

        double aux_north, aux_east, aux_down;
        ecef2Ned(x, y, z, &aux_north, &aux_east, &aux_down);

        *east = aux_east;
        *north = aux_north;
        *up = -aux_down;
      }

      void enu2Geodetic(const double east, const double north, const double up, double* latitude,
                        double* longitude, double* altitude)
      {
        // Local ENU position to geodetic coordinates

        const double aux_north = north;
        const double aux_east = east;
        const double aux_down = -up;
        double x, y, z;
        ned2Ecef(aux_north, aux_east, aux_down, &x, &y, &z);
        ecef2Geodetic(x, y, z, latitude, longitude, altitude);
      }

    private:
      inline Eigen::Matrix3d nRe(const double lat_radians, const double lon_radians)
      {
        const double sLat = sin(lat_radians);
        const double sLon = sin(lon_radians);
        const double cLat = cos(lat_radians);
        const double cLon = cos(lon_radians);

        Eigen::Matrix3d ret;
        ret(0, 0) = -sLat * cLon;
        ret(0, 1) = -sLat * sLon;
        ret(0, 2) = cLat;
        ret(1, 0) = -sLon;
        ret(1, 1) = cLon;
        ret(1, 2) = 0.0;
        ret(2, 0) = cLat * cLon;
        ret(2, 1) = cLat * sLon;
        ret(2, 2) = sLat;

        return ret;
      }

      inline
      double rad2Deg(const double radians)
      {
        return (radians / M_PI) * 180.0;
      }

      inline
      double deg2Rad(const double degrees)
      {
        return (degrees / 180.0) * M_PI;
      }

      double initial_latitude_;
      double initial_longitude_;
      double initial_altitude_;

      double initial_ecef_x_;
      double initial_ecef_y_;
      double initial_ecef_z_;

      Eigen::Matrix3d ecef_to_ned_matrix_;
      Eigen::Matrix3d ned_to_ecef_matrix_;

      bool haveReference_;

    }; // class NavSatConverter
    }
  }
}

#endif
