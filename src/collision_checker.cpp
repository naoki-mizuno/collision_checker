#include "collision_checker.h"

namespace msp {
CollisionChecker::CollisionChecker()
    : config_{}
{
}

CollisionChecker::CollisionChecker(const nav_msgs::OccupancyGrid& grid,
                                   const geometry_msgs::Polygon& robot)
    : config_{}
{
    set_grid(grid);
    set_robot(robot, config_.theta_resolution);
}

nav_msgs::OccupancyGrid
CollisionChecker::grid() const {
    return grid_;
}

nav_msgs::OccupancyGrid&
CollisionChecker::grid() {
    return grid_;
}

CollisionChecker::Config
CollisionChecker::config() const {
    return config_;
}

CollisionChecker::Config&
CollisionChecker::config() {
    return config_;
}

void
CollisionChecker::set_grid(const nav_msgs::OccupancyGrid& grid) {
    grid_ = grid;

    if (!robot_polygon_.points.empty()) {
        make_footprints();
    }
}

void
CollisionChecker::set_robot(const geometry_msgs::Polygon& robot,
                            const double theta_resolution /* = M_PI / 180 */) {
    robot_polygon_ = robot;
    config_.theta_resolution = theta_resolution;

    // Create footprints if grid is set
    if (grid_.info.resolution != 0) {
        make_footprints();
    }
}

void
CollisionChecker::set_config(const Config& config) {
    // Reconstruct robot footprints if theta resolution was updated
    if (config.theta_resolution != config_.theta_resolution) {
        config_.theta_resolution = config.theta_resolution;
        if (grid_.info.resolution != 0 && !robot_polygon_.points.empty()) {
            make_footprints();
        }
    }
    config_.occupancy_threshold = config.occupancy_threshold;
    config_.allow_unknown = config.allow_unknown;
}

bool
CollisionChecker::is_valid(const geometry_msgs::Pose& pose) {
    const auto x = pose.position.x;
    const auto y = pose.position.y;
    const auto theta = quaternion_to_theta(pose.orientation);
    return is_valid(x, y, theta);
}

bool
CollisionChecker::is_valid(const geometry_msgs::PoseStamped& pose_stamped) {
    // TODO: Coordinate transform into map frame
    return is_valid(pose_stamped.pose);
}

bool
CollisionChecker::is_valid(const double x,
                           const double y,
                           const double theta) {
    if (grid_.info.resolution == 0 && robot_polygon_.points.empty()) {
        // TODO: Raise exception?
        return false;
    }

    const auto res = grid_.info.resolution;
    // Theta in the range [0, 2 * M_PI)
    const auto norm_theta = theta < 0 ? theta + 2 * M_PI : theta;
    const auto theta_index = static_cast<unsigned>(norm_theta / config_.theta_resolution);
    const auto& footprint = robot_footprints_[theta_index];
    const auto footprint_size_x = footprint.size();
    const auto footprint_size_y = footprint.front().size();
    // TODO: Consider rotation of the map

    // Get X and Y offset (in cells) to bottom-left of footprint
    const auto m_canvas_offset_x = res * footprint_size_x / 2;
    const auto m_canvas_offset_y = res * footprint_size_y / 2;
    const auto m_x = x - m_canvas_offset_x;
    const auto m_y = y - m_canvas_offset_y;
    const auto index = pose_to_index(m_x, m_y);
    const auto c_canvas_offset_x = index.first;
    const auto c_canvas_offset_y = index.second;

    bool has_collision = false;
    // How many cells there are in the footprint
    const auto num_cells = footprint_size_x * footprint_size_y;

#pragma omp parallel for
    for (unsigned i = 0; i < num_cells; i++) {
        // Number of cells from the bottom-left of the footprint
        const auto c_footprint_x = i % footprint_size_x;
        const auto c_footprint_y = i / footprint_size_x;
        const auto c_x = c_footprint_x + c_canvas_offset_x;
        const auto c_y = c_footprint_y + c_canvas_offset_y;

        // If footprint is not occupied, no need to check
        if (!footprint[c_footprint_x][c_footprint_y]) {
            continue;
        }

        // If the point in footprint goes out of map, it's invalid
        if (c_x < 0 || c_y < 0 || c_x >= grid_.info.width || c_y >= grid_.info.height) {
            has_collision = true;
        }
        else {
            const auto idx = c_y * grid_.info.width + c_x;
            bool collision = grid_.data[idx] > config_.occupancy_threshold;
            // Unknown grid is considered occupied
            if (!config_.allow_unknown) {
                collision |= grid_.data[idx] == -1;
            }
#pragma omp atomic
            has_collision |= collision;
        }
    }
    return !has_collision;
}

nav_msgs::OccupancyGrid
CollisionChecker::get_footprint(const double x,
                                const double y,
                                const double theta) {
    const auto theta_index = static_cast<unsigned>(theta / config_.theta_resolution);
    const auto& footprint = robot_footprints_[theta_index];

    auto grid = nav_msgs::OccupancyGrid{};
    grid.header = grid_.header;
    grid.info.resolution = grid_.info.resolution;
    grid.info.width = footprint.size();
    grid.info.height = footprint.front().size();
    grid.data.resize(grid.info.width * grid.info.height);

    for (unsigned fp_x = 0; fp_x < footprint.size(); fp_x++) {
        for (unsigned fp_y = 0; fp_y < footprint.front().size(); fp_y++) {
            // Row-major-order
            const auto grid_idx = fp_y * grid.info.width + fp_x;
            if (footprint[fp_x][fp_y]) {
                grid.data[grid_idx] = 100;
            }
            else {
                grid.data[grid_idx] = 0;
            }
        }
    }

    grid.info.origin.position.x = x - (grid.info.resolution * grid.info.width / 2);
    grid.info.origin.position.y = y - (grid.info.resolution * grid.info.height / 2);
    grid.info.origin.position.z = grid_.info.origin.position.z;
    grid.info.origin.orientation.w = 1;

    return grid;
}

nav_msgs::OccupancyGrid
CollisionChecker::get_footprint(const geometry_msgs::Pose& pose) {
    const auto x = pose.position.x;
    const auto y = pose.position.y;
    const auto theta = quaternion_to_theta(pose.orientation);
    return get_footprint(x, y, theta);
}

bool
CollisionChecker::point_within_polygon(const geometry_msgs::Point32& p,
                                       const geometry_msgs::Polygon& polygon) {
    const auto poly = polygon.points;
    bool is_inside = false;

    // Reference: https://wrf.ecse.rpi.edu/Research/Short_Notes/pnpoly.html
    for (int i = 0, j = poly.size() - 1; i < poly.size(); j = i++) {
        const auto crossed =
            (
                (poly[i].y > p.y) != (poly[j].y > p.y)
            ) &&
            (
                p.x < (poly[j].x - poly[i].x) * (p.y - poly[i].y) / (poly[j].y - poly[i].y) + poly[i].x
            );

        if (crossed) {
            is_inside = !is_inside;
        }
    }
    return is_inside;
}

void
CollisionChecker::draw_line(const geometry_msgs::Point32& a,
                            const geometry_msgs::Point32& b,
                            Canvas& canvas) {
    // Traverse from p0 to p1 (from smaller X to larger X)
    auto p0 = a;
    auto p1 = b;
    if (a.x > b.x) {
        std::swap(p0, p1);
    }

    const auto res = grid_.info.resolution;
    const auto m_offset_x = res * canvas.size() / 2;
    const auto m_offset_y = res * canvas.front().size() / 2;
    // How much to increment the parameter t
    const auto step = 0.1 * res;
    const auto theta = std::atan2(p1.y - p0.y, p1.x - p0.x);

    auto t = 0.0;
    while (true) {
        const auto m_x = p0.x + t * std::cos(theta);
        const auto m_y = p0.y + t * std::sin(theta);
        const auto c_x = static_cast<unsigned>((m_x + m_offset_x) / res);
        const auto c_y = static_cast<unsigned>((m_y + m_offset_y) / res);

        canvas[c_x][c_y] = true;

        if (m_x >= p1.x) {
            break;
        }

        t += step;
    }
}

void
CollisionChecker::make_footprints() {
    // Make canvas 1 cell larger than longest edge of robot
    auto canvas_size = static_cast<unsigned>(
        std::ceil(2 * longest_edge(robot_polygon_) / grid_.info.resolution) + 1
    );
    // Make canvas size odd
    canvas_size += (canvas_size % 2 == 0 ? 1 : 0);

    const auto num_theta = static_cast<unsigned>(2 * M_PI / config_.theta_resolution);
    robot_footprints_.resize(num_theta);

    // Make a clean canvas
    for (auto& canvas : robot_footprints_) {
        canvas.resize(canvas_size);
        for (auto& row : canvas) {
            row.resize(canvas_size);
        }
    }

#pragma omp parallel for
    for (unsigned i = 0; i < robot_footprints_.size(); i++) {
        const auto theta = config_.theta_resolution * i;
        make_footprint(robot_polygon_, theta, robot_footprints_[i]);
    }
}

void
CollisionChecker::make_footprint(const geometry_msgs::Polygon& robot,
                                 const double theta,
                                 CollisionChecker::Canvas& canvas) {
    const auto COS = std::cos(theta);
    const auto SIN = std::sin(theta);

    // Rotate the robot's polygon by theta radians
    auto rotated_robot = geometry_msgs::Polygon{};
    rotated_robot.points.resize(robot.points.size());
    for (unsigned i = 0; i < robot.points.size(); i++) {
        const auto point = robot.points[i];
        const auto m_x = point.x * COS - point.y * SIN;
        const auto m_y = point.x * SIN + point.y * COS;
        rotated_robot.points[i].x = m_x;
        rotated_robot.points[i].y = m_y;
    }

    // Fill the polygon
    const auto res = grid_.info.resolution;
    const auto m_offset_x = res * canvas.size() / 2;
    const auto m_offset_y = res * canvas.front().size() / 2;
    for (unsigned i = 0; i < canvas.size(); i++) {
        for (unsigned j = 0; j < canvas.front().size(); j++) {
            // Cells -> Meters (use center of grid)
            const auto m_x = i * res + res / 2 - m_offset_x;
            const auto m_y = j * res + res / 2 - m_offset_y;

            auto p = geometry_msgs::Point32{};
            p.x = m_x;
            p.y = m_y;

            const auto is_inside = point_within_polygon(p, rotated_robot);
            canvas[i][j] = is_inside;
        }
    }

    // Trace each edge of the polygon
    rotated_robot.points.push_back(rotated_robot.points.front());
    for (unsigned i = 0; i < rotated_robot.points.size() - 1; i++) {
        const auto a = rotated_robot.points[i];
        const auto b = rotated_robot.points[i + 1];
        draw_line(a, b, canvas);
    }
}

CollisionChecker::Index
CollisionChecker::pose_to_index(const geometry_msgs::Pose& p) const {
    const auto& info = grid_.info;
    const auto m_x = p.position.x;
    const auto m_y = p.position.y;

    const auto c_x = (m_x - info.origin.position.x) / info.resolution;
    const auto c_y = (m_y - info.origin.position.y) / info.resolution;

    return Index{c_x, c_y};
}

CollisionChecker::Index
CollisionChecker::pose_to_index(const double m_x, const double m_y) const {
    auto pose = geometry_msgs::Pose{};
    pose.position.x = m_x;
    pose.position.y = m_y;
    return pose_to_index(pose);
}

geometry_msgs::Pose
CollisionChecker::index_to_pose(const unsigned c_x, const unsigned c_y) const {
    const auto& info = grid_.info;
    const auto m_x = c_x * info.resolution + info.origin.position.x;
    const auto m_y = c_y * info.resolution + info.origin.position.y;

    auto pose = geometry_msgs::Pose{};
    pose.position.x = m_x;
    pose.position.y = m_y;

    return pose;
}

geometry_msgs::Pose
CollisionChecker::index_to_pose(const Index& index) const {
    return index_to_pose(index.first, index.second);
}

double
CollisionChecker::quaternion_to_theta(const geometry_msgs::Quaternion& q) const {
    auto q_tf = tf2::Quaternion{};
    tf2::convert(q, q_tf);
    double r, p, y;
    tf2::Matrix3x3{q_tf}.getRPY(r, p, y);
    if (y < 0) {
        y += 2 * M_PI;
    }
    return y;
}

void
CollisionChecker::vizualize(const Canvas& canvas) {
    const auto size_x = canvas.size();
    const auto size_y = canvas.front().size();

    std::cout << "     Y" << std::endl;
    std::cout << "  +----->" << std::endl;
    std::cout << "  |" << std::endl;
    std::cout << "X |" << std::endl;
    std::cout << "  V" << std::endl;
    std::cout << "Res: " << grid_.info.resolution << " m/cell" << std::endl;
    std::cout << std::endl;

    for (int x = 0; x < size_x; x++) {
        for (int y = 0; y < size_y; y++) {
            const auto occupied = canvas[x][y];
            if (occupied) {
                std::cout << "O";
            }
            else if (x == size_x / 2) {
                std::cout << "-";
            }
            else if (y == size_y / 2) {
                std::cout << "|";
            }
            else {
                std::cout << " ";
            }
        }
        std::cout << std::endl;
    }
}

double
CollisionChecker::longest_edge(const geometry_msgs::Polygon& robot) const {
    const auto dist = [](const geometry_msgs::Point32& p) {
        return std::sqrt(std::pow(p.x, 2) + std::pow(p.y, 2));
    };
    const auto comp = [&](const geometry_msgs::Point32& a,
                         const geometry_msgs::Point32& b) {
        return dist(a) < dist(b);
    };

    const auto m = std::max_element(robot.points.begin(),
                                    robot.points.end(),
                                    comp);
    return dist(*m);
}
}