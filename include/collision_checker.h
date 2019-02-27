#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <geometry_msgs/Polygon.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Quaternion.h>
#include <nav_msgs/OccupancyGrid.h>

namespace msp {
class CollisionChecker {
public:
  CollisionChecker() = default;

  explicit CollisionChecker(const nav_msgs::OccupancyGrid& map,
                            const geometry_msgs::Polygon& robot);

  void
  set_grid(const nav_msgs::OccupancyGrid& grid);

  void
  set_robot(const geometry_msgs::Polygon& robot);

  nav_msgs::OccupancyGrid
  minkowski_sum(const nav_msgs::OccupancyGrid& map);

  void
  convolve();

  bool
  is_valid(const geometry_msgs::Pose& pose);

  bool
  is_valid(const geometry_msgs::PoseStamped& pose_stamped);

  bool
  is_valid(const double x, const double y, const double theta);

protected:
  // TODO: should we maintain a different data structure of FFTW?
  nav_msgs::OccupancyGrid grid_;

  geometry_msgs::Polygon robot_;

  double
  yaw_from_quaternion(const geometry_msgs::Quaternion& q);
};
}