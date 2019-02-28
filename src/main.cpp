#include "collision_checker.h"

#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>

void
cb_map(const nav_msgs::OccupancyGrid& grid) {
    msp::CollisionChecker cc;
    cc.set_grid(grid);
    ros::Duration{0.5}.sleep();
}

int
main(int argc, char* argv[]) {
    ros::init(argc, argv, "collision_checker_node");
    // fftwpp::fftw::maxthreads = static_cast<unsigned>(get_max_threads());
    // ROS_INFO_STREAM("Num threads : " << fftwpp::fftw::maxthreads);

    ros::NodeHandle nh;
    const auto sub = nh.subscribe("map", 1, cb_map);

    auto p = nh.advertise<nav_msgs::OccupancyGrid>("map_fft", 1, true);

    ros::spin();

    return 0;
}
