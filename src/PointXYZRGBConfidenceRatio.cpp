#define PCL_NO_PRECOMPILE
#include "confidence_mapping/PointXYZRGBConfidenceRatio.hpp"

#include <pcl/filters/extract_indices.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/crop_box.h>
#include <pcl/pcl_base.h>
#include <pcl/point_cloud.h>

template class pcl::PointCloud<pcl::PointXYZRGBConfidenceRatio>;
template class pcl::PCLBase<pcl::PointXYZRGBConfidenceRatio>;
template class pcl::VoxelGrid<pcl::PointXYZRGBConfidenceRatio>;
template void pcl::removeNaNFromPointCloud<pcl::PointXYZRGBConfidenceRatio>(
    const pcl::PointCloud<pcl::PointXYZRGBConfidenceRatio>& cloud_in,
    pcl::PointCloud<pcl::PointXYZRGBConfidenceRatio>& cloud_out,
    std::vector<int, std::allocator<int> >& index);
template class pcl::ExtractIndices<pcl::PointXYZRGBConfidenceRatio>;
template class pcl::PassThrough<pcl::PointXYZRGBConfidenceRatio>;
template class pcl::CropBox<pcl::PointXYZRGBConfidenceRatio>;

// ✅ Updated to include intensity in ostream
std::ostream& operator<<(std::ostream& os, const pcl::PointXYZRGBConfidenceRatio& p) {
  os << "(" << p.x << "," << p.y << "," << p.z
     << " - " << static_cast<int>(p.r)
     << "," << static_cast<int>(p.g)
     << "," << static_cast<int>(p.b)
     << " - confidence: " << p.confidence_ratio
     << " - intensity: " << p.intensity << ")";  // ✅ Added intensity
  return os;
}
