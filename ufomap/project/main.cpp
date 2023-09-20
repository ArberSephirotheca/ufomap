// For non-colored UFOMap
#include <ufo/map/occupancy_map.h>

int main()
{
  // 10 cm voxel size.
  double resolution = 0.1;

  // Maximum range to integrate, in meters.
  // Set to negative value to ignore maximum range.
  double max_range = 7.0;

  // The depth at which free space should be cleared.
  // A higher value significantly increases the integration speed
  // for smaller voxel sizes.
  ufo::map::DepthType integration_depth = 1;
  // Will free space at resolution * 2^(integration_depth) voxel size.

  // Some translation [x, y, z]
  ufo::math::Vector3 translation(0.0, 0.0, 0.0); 

  // Some rotation (w, x, y, z)
  ufo::math::Quaternion rotation(1.0, 0.0, 0.0, 0.0);

  ufo::math::Pose6 frame_origin(translation, rotation);

  ufo::math::Vector3 sensor_origin(translation);

// Create a UFOMap
  ufo::map::OccupancyMap map(resolution);

  // Point cloud
  ufo::map::PointCloud cloud;

  // Fill point cloud
  cloud.resize(1000);
  for (ufo::map::Point3& point : cloud)
  {
    point.x() = 1024 * rand () / (RAND_MAX + 1.0);
    point.y() = 1024 * rand () / (RAND_MAX + 1.0);
    point.z() = 1024 * rand () / (RAND_MAX + 1.0);
  }

  // Specify if the point cloud should be transformed in parallel or not.
  bool parallel = true;
  // Transform point cloud to correct frame
  cloud.transform(frame_origin, parallel);

  // Integrate point cloud into UFOMap
  map.insertPointCloudDiscrete(sensor_origin, cloud, max_range, integration_depth);

  return 0;
}