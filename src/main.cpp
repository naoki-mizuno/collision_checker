#include "collision_checker.h"

#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>

msp::CollisionChecker cc;
std::shared_ptr<ros::Publisher> pub_footprint;

void
cb_map(const nav_msgs::OccupancyGrid& grid) {
    cc.set_grid(grid);

    geometry_msgs::Polygon robot;
    robot.points.resize(5);
    robot.points[0].x = 0.7;
    robot.points[0].y = 0.0;
    robot.points[1].x = 0.5;
    robot.points[1].y = -0.2;
    robot.points[2].x = -0.3;
    robot.points[2].y = -0.2;
    robot.points[3].x = -0.3;
    robot.points[3].y = 0.2;
    robot.points[4].x = 0.5;
    robot.points[4].y = 0.2;
    for (auto& p : robot.points) {
        p.x *= 5;
        p.y *= 5;
    }
    cc.set_robot(robot);

    auto c = msp::CollisionChecker::Config{};
    c.theta_resolution = M_PI / 360;
    c.allow_unknown = true;
    c.occupancy_threshold = 0;
    cc.set_config(c);
}

void
cb_pose(const geometry_msgs::PoseWithCovarianceStamped& pwc) {
    const auto footprint = cc.get_footprint(pwc.pose.pose);
    pub_footprint->publish(footprint);

    std::cout << "is_valid: " << std::boolalpha << cc.is_valid(pwc.pose.pose) << std::endl;
}

int
main(int argc, char* argv[]) {
    ros::init(argc, argv, "collision_checker_node");

    ros::NodeHandle nh;
    const auto sub_map = nh.subscribe("map", 1, cb_map);
    const auto sub_pose = nh.subscribe("initialpose", 1, cb_pose);
    pub_footprint = std::make_shared<ros::Publisher>(
        nh.advertise<nav_msgs::OccupancyGrid>("footprint", 1, true)
    );

    ros::spin();

    return 0;
}
