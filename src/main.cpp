#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>

#include <collision_checker.h>

#include "fftwpp/Array.h"
#include "fftwpp/Complex.h"
#include "fftwpp/convolution.h"

ros::Publisher* pub;

void
draw_box(Array::array2<Complex>& canvas,
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
make_kernel(Array::array2<Complex>& canvas, const unsigned radius) {
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
make_impulses(Array::array2<Complex>& canvas,
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

void
cb_map(const nav_msgs::OccupancyGrid& map) {
    const auto start = ros::Time::now();
    ROS_INFO_STREAM("Received map");

    const auto mx = map.info.width;
    const auto my = 2 * map.info.height;
    const auto align = sizeof(Complex);
    // The occupancy grid map
    auto map_2d = Array::array2<Complex>{2 * mx - 1, my, align};
    const auto x_offset = map_2d.Nx() / 2;
    const auto y_offset = map_2d.Ny() / 2;
    // The kernel function
    auto kernel = Array::array2<Complex>{2 * mx - 1, my, align};
    make_kernel(kernel, 5);
    // Impulses
    auto impulses = Array::array2<Complex>{2 * mx - 1, my, align};
    make_impulses(impulses, 10, 2, M_PI / 4);

    // Occupancy grid (row-major-order) to 2D array
    for (unsigned i = 0; i < map.data.size(); i++) {
        const auto x = i % map.info.width;
        const auto y = i / map.info.width;
        // TODO: allow_unknown
        map_2d(x + x_offset, y + y_offset) = map.data[i] > 0 ? 1 : 0;
    }

    // const auto mat = impulses;
    // for (int x = 0; x < mat.Nx(); x++) {
    //     for (int y = 0; y < mat.Ny(); y++) {
    //         const auto val = mat(x, y);
    //         if (val.re > 0) {
    //             std::cout << "O";
    //         }
    //         else if (x == impulses.Nx() / 2) {
    //             std::cout << "-";
    //         }
    //         else if (y == impulses.Ny() / 2) {
    //             std::cout << "|";
    //         }
    //         else {
    //             std::cout << " ";
    //         }
    //     }
    //     std::cout << std::endl;
    // }

    ROS_INFO_STREAM("Convolution");
    auto C = fftwpp::ImplicitHConvolution2{mx, my};
    C.convolve(map_2d, kernel, true);
    C.convolve(map_2d, impulses, true);

    ROS_INFO_STREAM("To occupancy");
    auto map_new = map;
    const auto EPSILON = std::numeric_limits<float>::epsilon();
    for (unsigned y = 0; y < map_new.info.height; y++) {
        for (unsigned x = 0; x < map_new.info.width; x++) {
            const auto val = map_2d(x + x_offset, y + y_offset).real();
            const auto occupancy = val > EPSILON ? 100 : 0;

            const auto i = y * map_new.info.width + x;
            map_new.data[i] = static_cast<signed char>(occupancy);
        }
    }
    ROS_INFO_STREAM("Publishing");
    pub->publish(map_new);

    const auto finish = ros::Time::now();

    ROS_INFO_STREAM((finish - start).toSec() << " seconds");
}

int
main(int argc, char* argv[]) {
    ros::init(argc, argv, "collision_checker_node");

    ros::NodeHandle nh;
    const auto sub = nh.subscribe("map", 1, cb_map);

    auto p = nh.advertise<nav_msgs::OccupancyGrid>("map_fft", 1, true);
    pub = &p;

    //msp::CollisionChecker cc;
    ros::spin();

    return 0;
}
