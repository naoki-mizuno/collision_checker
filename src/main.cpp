#include "collision_checker.h"

#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>

#include <collision_checker/Check.h>
#include <collision_checker/SetConfig.h>

struct CheckerNode {
    msp::CollisionChecker cc_;
    ros::NodeHandle nh_;
    ros::Subscriber sub_map_;

    ros::ServiceServer check_service_;
    ros::ServiceServer config_service_;

    CheckerNode()
    {
        sub_map_ = nh_.subscribe("map", 1, &CheckerNode::cb_map, this);
        config_service_ = nh_.advertiseService(
            "set_config", &CheckerNode::set_config, this
        );
        check_service_ = nh_.advertiseService(
            "collision_check", &CheckerNode::collision_check, this
        );
    }

    void

    cb_map(const nav_msgs::OccupancyGrid& grid) {
        cc_.set_grid(grid);
    }

    bool
    set_config(collision_checker::SetConfig::Request& req,
               collision_checker::SetConfig::Response& res) {
        cc_.set_robot(req.robot);

        auto c = msp::CollisionChecker::Config{};
        c.theta_resolution = req.theta_resolution;
        c.allow_unknown = req.allow_unknown;
        c.occupancy_threshold = req.occupancy_threshold;
        cc_.set_config(c);

        return true;
    }

    bool
    collision_check(collision_checker::Check::Request& req,
                    collision_checker::Check::Response& res) {
        const auto pose = req.pose;

        res.footprint = cc_.get_footprint(pose);
        res.is_valid = cc_.is_valid(pose);

        return true;
    }
};


int
main(int argc, char* argv[]) {
    ros::init(argc, argv, "collision_checker_node");

    auto cn = CheckerNode{};

    ros::spin();

    return 0;
}
