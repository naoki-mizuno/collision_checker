#include "fftwpp/Array.h"
#include "fftwpp/Complex.h"
#include "fftwpp/convolution.h"

#include <ros/ros.h>

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
    static constexpr auto EPSILON = std::numeric_limits<float>::epsilon();

    CollisionChecker() = default;

    explicit CollisionChecker(const nav_msgs::OccupancyGrid& grid,
                              const geometry_msgs::Polygon& robot);

    nav_msgs::OccupancyGrid
    original_grid() const;

    void
    set_grid(const nav_msgs::OccupancyGrid& grid);

    void
    set_robot(const geometry_msgs::Polygon& robot);

    bool
    is_valid(const geometry_msgs::Pose& pose);

    bool
    is_valid(const geometry_msgs::PoseStamped& pose_stamped);

    bool
    is_valid(const double x, const double y, const double theta);

protected:
    nav_msgs::OccupancyGrid original_grid_;
    Array::array2<Complex> array_;

    geometry_msgs::Polygon robot_;

    void
    draw_box(Array::array2<Complex>& canvas,
             const int x1, const int y1,
             const int x2, const int y2);

    void
    make_kernel(Array::array2<Complex>& canvas, const unsigned radius);

    void
    make_impulses(Array::array2<Complex>& canvas,
                  const unsigned dist,
                  const int num,
                  const double theta);

    double
    yaw_from_quaternion(const geometry_msgs::Quaternion& q);

    void
    convert(const nav_msgs::OccupancyGrid& grid,
            Array::array2<Complex>& array);

    void
    convert(const Array::array2<Complex>& array,
            nav_msgs::OccupancyGrid& grid);

    void
    convert(const Array::array2<Complex>& array,
            nav_msgs::OccupancyGrid& grid,
            const unsigned width,
            const unsigned height);

private:
    void
    vizualize(const Array::array2<Complex>& canvas);
};
}