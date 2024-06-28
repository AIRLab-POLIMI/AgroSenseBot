#pragma once
#define PCL_NO_PRECOMPILE
#include <pcl/point_types.h>
#include <Eigen/Core>

namespace asb_ouster_ros {

  /** \brief point data type for the Ouster RNG19_RFL8_SIG16_NIR16 data profile. Members: x, y, z, intensity, t, reflectivity, ring, ambient, range.\n
   * \n
   * - intensity (Signal Photons) [16 bit unsigned int] - Signal intensity photons in the signal return measurement are reported.\n
   * - reflectivity (Calibrated Reflectivity) [8 bit unsigned int] - Sensor Signal Photons measurements are scaled based on measured range and sensor sensitivity at that range, providing an indication of target reflectivity.\n
   * - ambient (Near Infrared Photons) [16 bit unsigned int] - NIR photons related to natural environmental illumination are reported.\n
   * - range (Range) [19 bit unsigned int] - Range in millimeters, discretized to the nearest 1 millimeters with a maximum range of 524m. Note that range value will be set to 0 if out of range or if no detection is made.\n
   * \n
   * See: https://static.ouster.dev/sensor-docs/image_route1/image_route3/sensor_data/sensor-data.html#rng19-rfl8-sig16-nir16-return-profile\n
  */
struct EIGEN_ALIGN16 Point {
    PCL_ADD_POINT4D;
    float intensity;
    uint32_t t;
    uint16_t reflectivity;
    uint16_t ring;
    uint16_t ambient;
    uint32_t range;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};
}  // namespace asb_ouster_ros

// clang-format off
POINT_CLOUD_REGISTER_POINT_STRUCT(asb_ouster_ros::Point,
    (float, x, x)
    (float, y, y)
    (float, z, z)
    (float, intensity, intensity)
    (std::uint32_t, t, t)
    (std::uint16_t, reflectivity, reflectivity)
    (std::uint16_t, ring, ring)
    (std::uint16_t, ambient, ambient)
    (std::uint32_t, range, range)
)
// clang-format on
