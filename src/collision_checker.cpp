#include "collision_checker.h"

namespace msp {
CollisionChecker::CollisionChecker(const nav_msgs::OccupancyGrid& map,
                                   const geometry_msgs::Polygon& robot)
    : grid_{map}
    , robot_{robot}
{
  // Nothing to do here
}

void
CollisionChecker::set_grid(const nav_msgs::OccupancyGrid& grid) {
    grid_ = grid;

    // TODO: preprocess the map
}

void
CollisionChecker::set_robot(const geometry_msgs::Polygon& robot) {
    robot_ = robot;

    // TODO
}

nav_msgs::OccupancyGrid
CollisionChecker::minkowski_sum(const nav_msgs::OccupancyGrid& map) {
    // TODO
}

void
CollisionChecker::convolve() {
    // TODO
}

bool
CollisionChecker::is_valid(const geometry_msgs::Pose& pose) {
    const auto x = pose.position.x;
    const auto y = pose.position.y;
    const auto theta = yaw_from_quaternion(pose.orientation);
    return is_valid(x, y, theta);
}

bool
CollisionChecker::is_valid(const geometry_msgs::PoseStamped& pose_stamped) {
    return is_valid(pose_stamped.pose);
}

bool
CollisionChecker::is_valid(const double x,
                           const double y,
                           const double theta) {
    // TODO: Implement the meat!
}

double
CollisionChecker::yaw_from_quaternion(const geometry_msgs::Quaternion& q) {
    auto q_tf = tf2::Quaternion{};
    tf2::convert(q, q_tf);
    double r, p, y;
    tf2::Matrix3x3{q_tf}.getRPY(r, p, y);
    return y;
}
}