#pragma once

#include <pcl/pcl_macros.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace pcl {

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wpedantic"
struct _PointXYZRGBConfidenceRatio {  // NOLINT(cppcoreguidelines-pro-type-union-access)
  PCL_ADD_POINT4D;  // This adds x,y,z (and padding float[4])
  PCL_ADD_RGB;      // Adds r,g,b,a as a packed uint32_t rgba

  union {
    struct {
      float confidence_ratio;  // NOLINT(readability-identifier-naming)
      float intensity;         // ✅ Added intensity field
    };
    float data_c[4];  // Padding, used for SSE alignment
  };

  PCL_MAKE_ALIGNED_OPERATOR_NEW
} EIGEN_ALIGN16;  // Enforce SSE padding for correct memory alignment
#pragma GCC diagnostic pop

struct PointXYZRGBConfidenceRatio : public _PointXYZRGBConfidenceRatio {
  inline explicit PointXYZRGBConfidenceRatio(const _PointXYZRGBConfidenceRatio& p) : _PointXYZRGBConfidenceRatio() {
    x = p.x;
    y = p.y;
    z = p.z;
    data[3] = 1.0f;

    rgba = p.rgba;

    confidence_ratio = p.confidence_ratio;
    intensity = p.intensity;  // ✅ Copy intensity
  }

  inline explicit PointXYZRGBConfidenceRatio(float _confidence_ratio = 1.f, float _intensity = 0.f)
      : PointXYZRGBConfidenceRatio(0.f, 0.f, 0.f, 0, 0, 0, _confidence_ratio, _intensity) {}

  inline PointXYZRGBConfidenceRatio(std::uint8_t _r, std::uint8_t _g, std::uint8_t _b,
                                    float _confidence_ratio = 1.f, float _intensity = 0.f)
      : PointXYZRGBConfidenceRatio(0.f, 0.f, 0.f, _r, _g, _b, _confidence_ratio, _intensity) {}

  inline PointXYZRGBConfidenceRatio(float _x, float _y, float _z,
                                    float _confidence_ratio = 1.f, float _intensity = 0.f)
      : PointXYZRGBConfidenceRatio(_x, _y, _z, 0, 0, 0, _confidence_ratio, _intensity) {}

  inline PointXYZRGBConfidenceRatio(float _x, float _y, float _z,
                                    std::uint8_t _r, std::uint8_t _g, std::uint8_t _b,
                                    float _confidence_ratio = 1.f, float _intensity = 0.f)
      : _PointXYZRGBConfidenceRatio() {
    x = _x;
    y = _y;
    z = _z;
    data[3] = 1.0f;

    r = _r;
    g = _g;
    b = _b;
    a = 255;

    confidence_ratio = _confidence_ratio;
    intensity = _intensity;  // ✅ Set intensity
  }

  friend std::ostream& operator<<(std::ostream& os, const PointXYZRGBConfidenceRatio& p);
};

PCL_EXPORTS std::ostream& operator<<(std::ostream& os, const PointXYZRGBConfidenceRatio& p);

}  // namespace pcl

namespace confidence_mapping {
using PointCloudType = pcl::PointCloud<pcl::PointXYZRGBConfidenceRatio>;
}  // namespace confidence_mapping

// ✅ Updated registration macro to include intensity
POINT_CLOUD_REGISTER_POINT_STRUCT(pcl::_PointXYZRGBConfidenceRatio,
                                  (float, x, x)
                                  (float, y, y)
                                  (float, z, z)
                                  (std::uint32_t, rgba, rgba)
                                  (float, confidence_ratio, confidence_ratio)
                                  (float, intensity, intensity))

POINT_CLOUD_REGISTER_POINT_WRAPPER(pcl::PointXYZRGBConfidenceRatio, pcl::_PointXYZRGBConfidenceRatio)
