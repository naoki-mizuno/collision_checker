#include "collision_checker.h"

namespace msp {
CollisionChecker::CollisionChecker(const nav_msgs::OccupancyGrid& grid,
                                   const geometry_msgs::Polygon& robot)
{
  set_grid(grid);
  set_robot(robot);
}

nav_msgs::OccupancyGrid
CollisionChecker::original_grid() const {
    return original_grid_;
}

void
CollisionChecker::set_grid(const nav_msgs::OccupancyGrid& grid) {
    auto start = ros::Time::now();

    original_grid_ = grid;

    const auto mx = grid.info.width;
    const auto my = 2 * grid.info.height;
    const auto align = sizeof(Complex);

    auto C = fftwpp::ImplicitHConvolution2{mx, my};

    array_.Deallocate();
    array_.Allocate(2 * mx - 1, my, align);
    convert(grid, array_);

    // The kernel function
    auto kernel = Array::array2<Complex>{2 * mx - 1, my, align};
    make_kernel(kernel, 5);
    C.convolve(array_, kernel, true);

    // TODO: Loop for different impulses
    // Impulses
    auto impulses = Array::array2<Complex>{2 * mx - 1, my, align};
    make_impulses(impulses, 10, 2, M_PI / 4);
    C.convolve(array_, impulses, true);

    // TODO: Temporarily publish resulting map
    auto map_new = grid;
    convert(array_, map_new, map_new.info.width, map_new.info.height);
    ros::NodeHandle nh;
    auto pub = nh.advertise<nav_msgs::OccupancyGrid>("map_fft", 1, true);
    pub.publish(map_new);
    ros::spinOnce();

    auto finish = ros::Time::now();
    ROS_INFO_STREAM("Published in " << (finish - start).toSec() << " s");
}

void
CollisionChecker::set_robot(const geometry_msgs::Polygon& robot) {
    robot_ = robot;

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

void
CollisionChecker::draw_box(Array::array2<Complex>& canvas,
                           const int x1, const int y1,
                           const int x2, const int y2) {
    const auto xmin = std::min(x1, x2);
    const auto xmax = std::max(x1, x2);
    const auto ymin = std::min(y1, y2);
    const auto ymax = std::max(y1, y2);
    for (int x = xmin; x <= xmax; x++) {
        for (int y = ymin; y <= ymax; y++) {
            canvas(x + canvas.Nx() / 2, y + canvas.Ny() / 2) = 1;
        }
    }
}

void
CollisionChecker::make_kernel(Array::array2<Complex>& canvas,
                              const unsigned radius) {
    auto f = static_cast<int>(1 - radius);
    auto x = static_cast<int>(radius);
    auto y = 0;
    auto delta_up_left = static_cast<int>(-2 * radius);
    auto delta_up = 1;
    while (y <= x) {
        if (f >= 0) {
            draw_box(canvas, -x, -y, x, y);
            draw_box(canvas, -y, -x, y, x);
            x -= 1;
            delta_up_left += 2;
            f += delta_up_left;
        }
        y += 1;
        delta_up += 2;
        f += delta_up;
    }
}

void
CollisionChecker::make_impulses(Array::array2<Complex>& canvas,
                                const unsigned dist,
                                const int num,
                                const double theta) {
    const auto c = std::cos(theta);
    const auto s = std::sin(theta);
    // const auto lo = -std::floor(static_cast<double>(num) / 2);
    // const auto hi = std::ceil(static_cast<double>(num) / 2);
    const auto lo = 0;
    const auto hi = num - 1;

    for (int i = static_cast<int>(lo); i <= hi; i++) {
        const auto d = static_cast<double>(dist) * i;
        const auto x = static_cast<int>(std::round(c * d));
        const auto y = static_cast<int>(std::round(s * d));
        canvas(x + canvas.Nx() / 2, y + canvas.Ny() / 2) = 1;
    }
}

double
CollisionChecker::yaw_from_quaternion(const geometry_msgs::Quaternion& q) {
    auto q_tf = tf2::Quaternion{};
    tf2::convert(q, q_tf);
    double r, p, y;
    tf2::Matrix3x3{q_tf}.getRPY(r, p, y);
    return y;
}

void
CollisionChecker::convert(const nav_msgs::OccupancyGrid& grid,
                          Array::array2<Complex>& array) {
    const auto x_offset = array.Nx() / 2;
    const auto y_offset = array.Ny() / 2;

    // Occupancy grid (row-major-order) to 2D array
    for (unsigned i = 0; i < grid.data.size(); i++) {
        const auto x = i % grid.info.width;
        const auto y = i / grid.info.width;
        // TODO: allow_unknown
        array(x + x_offset, y + y_offset) = grid.data[i] > 0 ? 1 : 0;
    }
}

void
CollisionChecker::convert(const Array::array2<Complex>& array,
                          nav_msgs::OccupancyGrid& grid) {
    convert(array, grid, array.Nx(), array.Ny());
}

void
CollisionChecker::convert(const Array::array2<Complex>& array,
                          nav_msgs::OccupancyGrid& grid,
                          const unsigned width,
                          const unsigned height) {
    grid.info.width = width;
    grid.info.height = height;
    grid.data.resize(width * height);

    const auto x_offset = array.Nx() / 2;
    const auto y_offset = array.Ny() / 2;

    for (unsigned y = 0; y < height; y++) {
        for (unsigned x = 0; x < width; x++) {
            const auto val = array(x + x_offset, y + y_offset).real();
            const auto occupancy_val = val > EPSILON ? 100 : 0;

            const auto i = y * width + x;
            grid.data[i] = static_cast<signed char>(occupancy_val);
        }
    }
}

void
CollisionChecker::vizualize(const Array::array2<Complex>& canvas) {
    for (int x = 0; x < canvas.Nx(); x++) {
        for (int y = 0; y < canvas.Ny(); y++) {
            const auto val = canvas(x, y);
            if (val.re > 0) {
                std::cout << "O";
            }
            else if (x == canvas.Nx() / 2) {
                std::cout << "-";
            }
            else if (y == canvas.Ny() / 2) {
                std::cout << "|";
            }
            else {
                std::cout << " ";
            }
        }
        std::cout << std::endl;
    }
}
}