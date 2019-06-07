#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <geometry_msgs/Polygon.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Quaternion.h>
#include <nav_msgs/OccupancyGrid.h>

#ifdef _OPENMP
#include <omp.h>
#endif

/**
 * Terminologies:
 *
 * Size: unit is in cells (index)
 * Length: unit is in meters
 *
 * Variable prefix
 * c_ prefix: unit is in cells
 * m_ prefix: unit is in meters
 */

namespace msp {
class CollisionChecker {
public:
    using Index = std::pair<int, int>;

    /**
     *         Y
     *       ---->
     *
     *      +-----+
     *   |  | | | |
     * X |  +-----+
     *   |  | | | |
     *   V  +-----+
     *
     * true if occupied, false otherwise
     */
    using Canvas = std::vector<std::vector<bool>>;

    struct Config {
        /**
         * Resolution of yaw angle
         */
        double theta_resolution = M_PI / 180;
        /**
         * If occupancy is above this threshold, consider cell to be occupied
         */
        double occupancy_threshold = 10;
        /**
         * Whether to consider unknown grids as occupied or not
         */
        bool allow_unknown = true;
    };

    CollisionChecker();

    explicit CollisionChecker(const nav_msgs::OccupancyGrid& grid,
                              const geometry_msgs::Polygon& robot);

    nav_msgs::OccupancyGrid
    grid() const;

    nav_msgs::OccupancyGrid&
    grid();

    Config
    config() const;

    Config&
    config();

    void
    set_grid(const nav_msgs::OccupancyGrid& grid);

    void
    set_robot(const geometry_msgs::Polygon& robot,
              const double theta_resolution = M_PI / 180);

    void
    set_config(const Config& config);

    /**
     * Checks whether the robot is collision free in the given configuration
     *
     * Assumes the same coordinate frame as the grid
     *
     * @param x X position of the robot in meters
     * @param y Y position of the robot in meters
     * @param theta rotation of the robot in radians
     * @return true if robot is collision free, false otherwise
     */
    bool
    is_valid(const geometry_msgs::Pose& pose);

    /**
     * Checks whether the robot is collision free in the given configuration
     *
     * Assumes the same coordinate frame as the grid
     *
     * @param x X position of the robot in meters
     * @param y Y position of the robot in meters
     * @param theta rotation of the robot in radians
     * @return true if robot is collision free, false otherwise
     */
    bool
    is_valid(const geometry_msgs::PoseStamped& pose_stamped);

    /**
     * Checks whether the robot is collision free in the given configuration
     *
     * Assumes the same coordinate frame as the grid
     *
     * @param x X position of the robot in meters
     * @param y Y position of the robot in meters
     * @param theta rotation of the robot in radians
     * @return true if robot is collision free, false otherwise
     */
    bool
    is_valid(const double x, const double y, const double theta);

    /**
     * Return the robot footprint as a nav_msgs/OccupancyGrid message
     * Mainly used for visualization.
     * @param x
     * @param y
     * @param theta
     * @return
     */
    nav_msgs::OccupancyGrid
    get_footprint(const double x, const double y, const double theta);

    /**
     * Return the robot footprint as a nav_msgs/OccupancyGrid message
     * Mainly used for visualization.
     * @param pose
     * @return
     */
    nav_msgs::OccupancyGrid
    get_footprint(const geometry_msgs::Pose& pose);

protected:
    nav_msgs::OccupancyGrid grid_;

    /**
     * Footprint of the robot for each theta
     */
    std::vector<Canvas> robot_footprints_;

    geometry_msgs::Polygon robot_polygon_;

    Config config_;

    /**
     * Checks whether the given point is within the given polygon
     * Reference: https://wrf.ecse.rpi.edu/Research/Short_Notes/pnpoly.html
     * @param p
     * @param polygon
     * @return
     */
    static
    bool
    point_within_polygon(const geometry_msgs::Point32& p,
                         const geometry_msgs::Polygon& polygon);

    void
    draw_line(const geometry_msgs::Point32& a,
              const geometry_msgs::Point32& b,
              Canvas& canvas);

    /**
     * Create robot footprints for all possible thetas (yaw angles)
     */
    void
    make_footprints();

    /**
     * Create a robot footprint for the given theta
     */
    void
    make_footprint(const geometry_msgs::Polygon& robot,
                   const double theta,
                   Canvas& canvas);

    Index
    pose_to_index(const geometry_msgs::Pose& p) const;

    Index
    pose_to_index(const double m_x, const double m_y) const;

    geometry_msgs::Pose
    index_to_pose(const unsigned c_x, const unsigned c_y) const;

    geometry_msgs::Pose
    index_to_pose(const Index& index) const;

    /**
     * Converts the given quaternion to yaw in the range [0, 2 * M_PI)
     * @param q Quaternion
     * @return yaw angle in radians whose range is [0, 2 * M_PI)
     */
    double
    quaternion_to_theta(const geometry_msgs::Quaternion& q) const;

private:
    /**
     * Find the longest edge in the given robot polygon
     * Used to determine the size of the canvas big enough to fit the whole
     * robot
     * @param robot
     * @return
     */
    double
    longest_edge(const geometry_msgs::Polygon& robot) const;

    void
    vizualize(const Canvas& canvas);
};
}