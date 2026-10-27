// Released under GPLv3: https://www.gnu.org/licenses/gpl-3.0.html
// Author: Claude Sammut
// Modified for real TurtleBot wall following

#include "wall_follower/wall_follower.hpp"

#include <memory>
#include <cmath>

using namespace std::chrono_literals;

WallFollower::WallFollower()
: Node("wall_follower_node")
{
    /************************************************************
    ** Initialise variables
    ************************************************************/
    for (int i = 0; i < 12; i++)
        scan_data_[i] = 0.0;

    robot_pose_ = 0.0;
    near_start = false;

    /************************************************************
    ** Initialise ROS publishers and subscribers
    ************************************************************/
    auto qos = rclcpp::QoS(rclcpp::KeepLast(10));

    // Publisher
    cmd_vel_pub_ =
        this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", qos);

    // Laser subscriber
    scan_sub_ =
        this->create_subscription<sensor_msgs::msg::LaserScan>(
            "scan",
            rclcpp::SensorDataQoS(),
            std::bind(
                &WallFollower::scan_callback,
                this,
                std::placeholders::_1));

    // Odometry subscriber
    odom_sub_ =
        this->create_subscription<nav_msgs::msg::Odometry>(
            "odom",
            qos,
            std::bind(
                &WallFollower::odom_callback,
                this,
                std::placeholders::_1));

    /************************************************************
    ** Timer
    ************************************************************/
    update_timer_ =
        this->create_wall_timer(
            10ms,
            std::bind(&WallFollower::update_callback, this));

    RCLCPP_INFO(
        this->get_logger(),
        "Wall follower node has been initialised");
}

WallFollower::~WallFollower()
{
    RCLCPP_INFO(
        this->get_logger(),
        "Wall follower node has been terminated");
}


/********************************************************************************
** [NEW V4] Current odometry position for one temporary wall-end reference.
** [MODIFIED V13] No map is built; return detection is tracked in odometry.
********************************************************************************/
namespace
{
    double wf_robot_x_ = 0.0, wf_robot_y_ = 0.0;
    bool wf_odom_valid_ = false;
    // [NEW V11] Gate faster motion on fresh measured odometry.
    double wf_measured_speed_ = 0.0;
    // [NEW V13] Read-only return progress for slowing/logging in the controller.
    bool wf_return_armed_ = false;
    double wf_start_distance_ = 0.0, wf_path_length_ = 0.0;
    double wf_return_radius_ = 0.35;
    std::chrono::steady_clock::time_point wf_odom_time_;
}

/********************************************************************************
** Odometry callback
********************************************************************************/

#define START_RANGE 0.35 // [MODIFIED V13] Default circular return radius, metres.

void WallFollower::odom_callback(
    const nav_msgs::msg::Odometry::SharedPtr msg)
{
    /****************************************************************
    ** [MODIFIED V13] Keep the FIRST starting pose for this process.
    ** Arm only after a real departure, enough travel and elapsed motion.
    ** A return remains stopped until the node is restarted.
    ****************************************************************/
    static bool first = true, departed = false;
    static double previous_x = 0.0, previous_y = 0.0, run_time = 0.0;
    static double path_x = 0.0, path_y = 0.0; // [NEW V13] Noise-resistant path keyframe.
    static auto previous_time = std::chrono::steady_clock::now();
    static const double min_return_path = std::fmax(1.0,
        this->declare_parameter<double>("min_return_path", 2.5));
    static const double min_return_time = std::fmax(0.0,
        this->declare_parameter<double>("min_return_time", 8.0));
    static const double return_radius = std::fmax(0.15, std::fmin(0.60,
        this->declare_parameter<double>("return_radius", START_RANGE)));
    wf_return_radius_ = return_radius;

    tf2::Quaternion q(
        msg->pose.pose.orientation.x,
        msg->pose.pose.orientation.y,
        msg->pose.pose.orientation.z,
        msg->pose.pose.orientation.w);

    tf2::Matrix3x3 m(q);

    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);

    robot_pose_ = yaw;

    double current_x = msg->pose.pose.position.x;
    double current_y = msg->pose.pose.position.y;

    // [MODIFIED V13] Position/speed snapshot and return progress.
    wf_robot_x_ = current_x;
    wf_robot_y_ = current_y;
    wf_odom_valid_ = true;
    wf_measured_speed_ = std::fabs(msg->twist.twist.linear.x); // [NEW V11]
    wf_odom_time_ = std::chrono::steady_clock::now(); // [NEW V11]

    auto now = std::chrono::steady_clock::now();
    if (first)
    {
        start_x = previous_x = current_x;
        start_y = previous_y = current_y;
        path_x = current_x; path_y = current_y;
        previous_time = now;
        first = false;
        wf_start_distance_ = wf_path_length_ = 0.0;
        wf_return_armed_ = false;
        return;
    }

    double elapsed = std::chrono::duration<double>(now - previous_time).count();
    previous_time = now;
    double dx = current_x - previous_x, dy = current_y - previous_y;
    double step = std::hypot(dx, dy);
    wf_start_distance_ = std::hypot(current_x - start_x, current_y - start_y);

    // [NEW V13] A large odometry discontinuity is not a return crossing.
    // Ignore that segment and require a new departure before arming again.
    if (step > 0.50)
    {
        departed = false;
        wf_return_armed_ = false;
        wf_path_length_ = run_time = 0.0;
        previous_x = current_x; previous_y = current_y;
        path_x = current_x; path_y = current_y;
        RCLCPP_INFO(this->get_logger(), "return_check=odom_jump_rearm");
        return;
    }
    // [NEW V13] Small stationary pose jitter must not accumulate metres of travel.
    double path_step = std::hypot(current_x - path_x, current_y - path_y);
    if (path_step >= 0.03)
    {
        wf_path_length_ += path_step;
        path_x = current_x; path_y = current_y;
    }
    if (elapsed > 0.0 && elapsed < 1.0) run_time += elapsed;
    if (wf_start_distance_ > std::fmax(0.70, return_radius + 0.30))
        departed = true;
    wf_return_armed_ = departed && wf_path_length_ >= min_return_path &&
                       run_time >= min_return_time;

    /****************************************************************
    ** [NEW V13] Closest point of the last odometry segment to the start.
    ** Detect a fast crossing even if neither sample lands inside the circle.
    ** Odometry drift beyond return_radius still needs external localisation.
    ****************************************************************/
    double fraction = step > 0.000001 ?
        ((start_x - previous_x) * dx + (start_y - previous_y) * dy) / (step * step) : 0.0;
    fraction = std::fmax(0.0, std::fmin(1.0, fraction));
    double crossing_distance = std::hypot(previous_x + fraction * dx - start_x,
                                          previous_y + fraction * dy - start_y);
    previous_x = current_x; previous_y = current_y;
    if (!near_start && wf_return_armed_ && crossing_distance <= return_radius)
    {
        near_start = true;
        update_cmd_vel(0.0, 0.0); // [NEW V13] Stop immediately, then keep stopping in timer.
        RCLCPP_INFO(this->get_logger(), "stop=returned_to_start distance=%.2f path=%.2f",
            crossing_distance, wf_path_length_);
    }
    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
        "return_check armed=%d distance=%.2f path=%.2f elapsed=%.1f radius=%.2f stopped=%d",
        int(wf_return_armed_), wf_start_distance_, wf_path_length_, run_time,
        return_radius, int(near_start));

}


/********************************************************************************
** Laser scan callback
********************************************************************************/

#define BEAM_WIDTH 15

/********************************************************************************
** [NEW] Shared local wall measurements (single-node, single-thread program)
** Stored in this .cpp only: no changes to wall_follower.hpp or build files.
********************************************************************************/
namespace
{
    /****************************************************************
    ** [NEW V7] Distances are measured from the laser, in metres.
    ** [MODIFIED V10] Parallel spacing is 0.35 m; endpoint arc stays 0.38 m.
    ****************************************************************/
    constexpr double wf_target_distance_ = 0.35; // [MODIFIED V10] 0.40 -> 0.35 m
    /****************************************************************
    ** [NEW V8] One temporary endpoint/line reference during a thin-wall
    ** U-turn. Reject other wall lines until this manoeuvre is complete.
    ****************************************************************/
    constexpr double wf_round_radius_ = 0.38;
    bool wf_round_lock_ = false;
    double wf_round_x_ = 0.0, wf_round_y_ = 0.0, wf_round_yaw_ = 0.0;
    /****************************************************************
    ** [NEW V9] Right-wall/corridor measurements; no map or header change.
    ** 0.20 m is the existing laser-centred half-width approximation.
    ****************************************************************/
    bool wf_right_valid_ = false, wf_corridor_valid_ = false;
    bool wf_left_wall_only_ = false, wf_right_wall_only_ = false;
    double wf_right_distance_ = 0.0, wf_right_angle_ = 0.0;
    double wf_right_min_x_ = 0.0, wf_right_max_x_ = 0.0;
    double wf_corridor_front_ = 0.0;
    bool wf_left_exit_ = false;
    double wf_exit_angle_ = 0.0;
    double wf_forward_clearance_ = 0.0;
    bool wf_wall_valid_ = false, wf_wall_end_visible_ = false;
    double wf_wall_distance_ = 0.0, wf_wall_angle_ = 0.0;
    double wf_wall_min_x_ = 0.0, wf_wall_max_x_ = 0.0;
    double wf_wall_end_x_ = 0.0, wf_wall_end_y_ = 0.0;
    unsigned long wf_scan_sequence_ = 0;
    double wf_scan_period_ = 0.20; // [NEW V11] Seconds between scan timestamps.
    double wf_sensor_time_ = 0.0; // [NEW V6] Scan time for pause-safe manoeuvre timing.
    bool wf_scan_valid_ = false;
    std::chrono::steady_clock::time_point wf_scan_time_;
}

void WallFollower::scan_callback(
    const sensor_msgs::msg::LaserScan::SharedPtr msg)
{
    /****************************************************************
    ** [NEW] Keep the original 360-point scan assumption; reject other
    ** formats before accessing ranges. No header changes are needed.
    ****************************************************************/
    wf_scan_valid_ = msg->ranges.size() == 360 &&
        std::fabs(msg->angle_min) < 0.0001 &&
        std::fabs(msg->angle_increment - 3.141592653589793 / 180.0) < 0.0001;
    if (!wf_scan_valid_)
    {
        wf_wall_valid_ = false;
        return;
    }
    wf_scan_time_ = std::chrono::steady_clock::now();
    // [NEW V11] Estimate sensor update delay; ignore resets/long pauses.
    double scan_stamp = msg->header.stamp.sec + msg->header.stamp.nanosec * 1e-9;
    double scan_period = scan_stamp - wf_sensor_time_;
    if (wf_scan_sequence_ > 0 && scan_period > 0.01 && scan_period < 1.0)
        wf_scan_period_ += 0.30 * (scan_period - wf_scan_period_);
    wf_sensor_time_ = scan_stamp;

    uint16_t scan_angle[12] =
    {
        0,
        30,
        60,
        90,
        120,
        150,
        180,
        210,
        240,
        270,
        300,
        330
    };

    /****************************************************************
    ** FRONT direction
    **
    ** 0 degrees wraps around:
    ** 345 -> 359 and 0 -> 14
    ****************************************************************/

    double closest = msg->range_max;

    for (
        int angle = 360 - BEAM_WIDTH;
        angle < 360;
        angle++)
    {
        double r = msg->ranges.at(angle);

        // Ignore invalid readings such as 0.0
        if (
            r >= msg->range_min &&
            r <= msg->range_max &&
            r < closest)
        {
            closest = r;
        }
    }

    for (
        int angle = 0;
        angle < BEAM_WIDTH;
        angle++)
    {
        double r = msg->ranges.at(angle);

        if (
            r >= msg->range_min &&
            r <= msg->range_max &&
            r < closest)
        {
            closest = r;
        }
    }

    scan_data_[0] = closest;


    /****************************************************************
    ** Remaining directions
    ****************************************************************/

    for (int i = 1; i < 12; i++)
    {
        closest = msg->range_max;

        for (
            int angle = scan_angle[i] - BEAM_WIDTH;
            angle < scan_angle[i] + BEAM_WIDTH;
            angle++)
        {
            double r = msg->ranges.at(angle);

            // Ignore invalid zero / too-close readings
            if (
                r >= msg->range_min &&
                r <= msg->range_max &&
                r < closest)
            {
                closest = r;
            }
        }

        scan_data_[i] = closest;
    }

    /****************************************************************
    ** [NEW V7] Check a passage, rather than one long diagonal ray.
    ** Keep even single cylinder returns; no point-count filter here.
    ** A candidate must have a clear 0.75 m by 0.40 m local corridor.
    ** This is a laser-centred clearance approximation, not a body model.
    ****************************************************************/
    wf_left_exit_ = false;
    wf_forward_clearance_ = msg->range_max;
    for (int i = 0; i < 360; i++)
    {
        double r = msg->ranges.at(i);
        if (!std::isfinite(r) || r < msg->range_min || r > msg->range_max)
            continue;
        double a = msg->angle_min + i * msg->angle_increment;
        double x = r * std::cos(a), y = r * std::sin(a);
        if (x > 0.0 && std::fabs(y) < 0.20 && x < wf_forward_clearance_)
            wf_forward_clearance_ = x;
    }
    for (int heading = 40; heading <= 110; heading += 5)
    {
        double centre = msg->ranges.at(heading);
        bool open = (std::isinf(centre) && centre > 0.0) ||
            (std::isfinite(centre) && centre >= 0.80 && centre <= msg->range_max);
        if (!open) continue;
        double direction = msg->angle_min + heading * msg->angle_increment;
        for (int i = 0; i < 360 && open; i++)
        {
            double r = msg->ranges.at(i);
            double a = msg->angle_min + i * msg->angle_increment - direction;
            double angle = std::atan2(std::sin(a), std::cos(a));
            // Missing/invalid data in the intended direction is not free space.
            if (!(std::isinf(r) && r > 0.0) &&
                (!std::isfinite(r) || r < msg->range_min || r > msg->range_max))
            {
                if (std::fabs(angle) < 0.27) open = false;
                continue;
            }
            if (!std::isfinite(r)) continue;
            double along = r * std::cos(a), across = r * std::sin(a);
            if (along > 0.0 && along < 0.75 && std::fabs(across) < 0.20)
                open = false;
        }
        if (open && (!wf_left_exit_ ||
            std::fabs(direction - 1.570796326794897) <
            std::fabs(wf_exit_angle_ - 1.570796326794897)))
        {
            wf_left_exit_ = true;
            wf_exit_angle_ = direction;
        }
    }

    /****************************************************************
    ** [NEW] Fit a continuous left-wall segment
    ** Original 12-sector measurements above remain unchanged.
    ** Assumption: 360 readings, index 0 forward, index 90 left.
    ****************************************************************/
    wf_wall_valid_ = false;
    wf_right_valid_ = wf_corridor_valid_ = false; // [NEW V9]
    wf_left_wall_only_ = wf_right_wall_only_ = false;
    wf_wall_end_visible_ = false;
    wf_scan_sequence_++;

    bool fitting_right = false; // [NEW V9] Reuse the same segment fitter.
    double right_score = 1000.0;
    double best_score = 1000.0;
    double sx = 0.0, sy = 0.0, sxx = 0.0, sxy = 0.0, syy = 0.0;
    double min_x = 1000.0, max_x = -1000.0;
    double last_x = 0.0, last_y = 0.0, last_r = 0.0;
    double end_y = 0.0;
    int count = 0, end_index = 0;

    // Evaluate one continuous group. Reject short or curved groups.
    auto fit_group = [&]()
    {
        if (count < 8 || max_x - min_x < 0.30)
            return;

        double xx = sxx - sx * sx / count;
        double xy = sxy - sx * sy / count;
        double yy = syy - sy * sy / count;
        if (xx < 0.001)
            return;

        // y = slope*x + intercept; suitable for near-parallel walls.
        double slope = xy / xx;
        double intercept = (sy - slope * sx) / count;
        double angle = std::atan(slope);
        double distance = (fitting_right ? -intercept : intercept) /
                          std::sqrt(1.0 + slope * slope); // [MODIFIED V9]
        double residual = (yy - xy * xy / xx) /
                          (count * (1.0 + slope * slope));

        if (distance < 0.15 || distance > 1.20 ||
            std::fabs(angle) > 0.70 || residual > 0.0009 || // [MODIFIED V4] Allow modest laser noise.
            min_x > 0.20 || max_x < -0.15) // [MODIFIED V4] Accept shorter visible walls.
            return;

        /************************************************************
        ** [NEW V9] Keep a right segment too, with its actual signed slope.
        ** Short cylinder arcs still fail the length/straightness tests.
        ************************************************************/
        if (fitting_right)
        {
            double score = distance + 0.15 * std::fabs(angle);
            if (score < right_score)
            {
                right_score = score;
                wf_right_valid_ = true;
                wf_right_distance_ = distance;
                wf_right_angle_ = angle;
                wf_right_min_x_ = min_x; wf_right_max_x_ = max_x;
            }
            return;
        }

        /************************************************************
        ** [NEW V8] A similar distance alone does not identify a wall.
        ** Match direction AND distance to the remembered endpoint.
        ** Allow up to 12 cm for wall thickness and fitting/odometry error.
        ************************************************************/
        if (wf_round_lock_)
        {
            double direction_error = angle + robot_pose_ - wf_round_yaw_;
            double dx = wf_round_x_ - wf_robot_x_;
            double dy = wf_round_y_ - wf_robot_y_;
            double c = std::cos(robot_pose_), sn = std::sin(robot_pose_);
            double tip_x = c * dx + sn * dy, tip_y = -sn * dx + c * dy;
            double line_error = std::fabs(tip_y - slope * tip_x - intercept) /
                                std::sqrt(1.0 + slope * slope);
            if (std::fabs(std::sin(direction_error)) > std::sin(0.20) ||
                line_error > 0.12)
                return;
        }

        // [MODIFIED V7] Rank segments with the new parallel-wall distance.
        double score = std::fabs(distance - wf_target_distance_) + 0.15 * std::fabs(angle);
        if (score >= best_score)
            return;

        best_score = score;
        wf_wall_valid_ = true;
        wf_wall_distance_ = distance;
        wf_wall_angle_ = angle;
        wf_wall_min_x_ = min_x;
        wf_wall_max_x_ = max_x;
        wf_wall_end_x_ = max_x;
        wf_wall_end_y_ = end_y;
        wf_wall_end_visible_ = false;

        // Check three rays just beyond the forward end of this wall.
        if (end_index > 32)
        {
            int open = 0;
            for (int k = 1; k <= 3; k++)
            {
                double a = msg->angle_min + (end_index - k) * msg->angle_increment;
                double denominator = std::sin(a) - slope * std::cos(a);
                if (denominator <= 0.10)
                    continue;

                double expected = intercept / denominator;
                double r = msg->ranges.at(end_index - k);
                if (expected + 0.25 >= msg->range_max)
                    continue;

                // Positive infinity means no return; NaN/zero is not evidence.
                if ((std::isinf(r) && r > 0.0) ||
                    (std::isfinite(r) && r >= msg->range_min &&
                     r <= msg->range_max && r > expected + 0.25))
                    open++;
            }
            wf_wall_end_visible_ = (open == 3);
        }
    };

    for (int i = 30; i <= 150; i++)
    {
        double r = msg->ranges.at(i);
        bool valid = std::isfinite(r) && r >= msg->range_min &&
                     r <= msg->range_max && r <= 2.0;
        double a = msg->angle_min + i * msg->angle_increment;
        double x = valid ? r * std::cos(a) : 0.0;
        double y = valid ? r * std::sin(a) : 0.0;

        if (!valid || (count > 0 &&
            std::hypot(x - last_x, y - last_y) >
            0.06 + 3.0 * last_r * msg->angle_increment))
        {
            fit_group();
            sx = sy = sxx = sxy = syy = 0.0;
            min_x = 1000.0;
            max_x = -1000.0;
            count = 0;
        }
        if (!valid)
            continue;

        count++;
        sx += x; sy += y;
        sxx += x * x; sxy += x * y; syy += y * y;
        if (x < min_x) min_x = x;
        if (x > max_x)
        {
            max_x = x;
            end_y = y;
            end_index = i;
        }
        last_x = x; last_y = y; last_r = r;
    }
    fit_group();
    /****************************************************************
    ** [NEW V9] Measure the right wall using the existing fitter.
    ****************************************************************/
    fitting_right = true;
    sx = sy = sxx = sxy = syy = 0.0;
    min_x = 1000.0; max_x = -1000.0; count = 0;
    for (int i = 210; i <= 330; i++)
    {
        double r = msg->ranges.at(i);
        bool valid = std::isfinite(r) && r >= msg->range_min &&
                     r <= msg->range_max && r <= 2.0;
        double a = msg->angle_min + i * msg->angle_increment;
        double x = valid ? r * std::cos(a) : 0.0;
        double y = valid ? r * std::sin(a) : 0.0;

        if (!valid || (count > 0 &&
            std::hypot(x - last_x, y - last_y) >
            0.06 + 3.0 * last_r * msg->angle_increment))
        {
            fit_group();
            sx = sy = sxx = sxy = syy = 0.0;
            min_x = 1000.0;
            max_x = -1000.0;
            count = 0;
        }
        if (!valid)
            continue;

        count++;
        sx += x; sy += y;
        sxx += x * x; sxy += x * y; syy += y * y;
        if (x < min_x) min_x = x;
        if (x > max_x)
        {
            max_x = x;
            end_y = y;
            end_index = i;
        }
        last_x = x; last_y = y; last_r = r;
    }
    fit_group();
    /****************************************************************
    ** [NEW V9] Two long, nearly parallel walls define a local corridor.
    ** Do not erase short returns: a point inside either wall line is
    ** still an obstacle. Front clearance is measured along the corridor.
    ****************************************************************/
    wf_corridor_valid_ = wf_wall_valid_ && wf_right_valid_ &&
        std::fabs(wf_wall_angle_ - wf_right_angle_) < 0.12 &&
        std::fabs(wf_wall_angle_) < 0.35 &&
        wf_wall_min_x_ < -0.08 && wf_wall_max_x_ > 0.08 &&
        wf_right_min_x_ < -0.08 && wf_right_max_x_ > 0.08 &&
        wf_wall_distance_ > 0.20 && wf_right_distance_ > 0.20 &&
        wf_wall_distance_ + wf_right_distance_ < 0.90;
    double corridor_angle = 0.5 * (wf_wall_angle_ + wf_right_angle_);
    wf_corridor_front_ = msg->range_max;
    int close_left = 0, close_right = 0, wall_left = 0, wall_right = 0;
    for (int i = 0; i < 360; i++)
    {
        double r = msg->ranges.at(i);
        if (!std::isfinite(r) || r < msg->range_min || r > msg->range_max)
            continue;
        double a = msg->angle_min + i * msg->angle_increment;
        double x = r * std::cos(a), y = r * std::sin(a);
        // [NEW V9] During rounding, the remembered wall/tip can remain
        // close even when it is too oblique for the near-parallel fitter.
        // Only explain returns on that wall behind its endpoint, not new objects.
        bool on_round_wall = false;
        if (wf_round_lock_ && r > 0.23)
        {
            double c = std::cos(robot_pose_), sn = std::sin(robot_pose_);
            double dx = wf_robot_x_ + c * x - sn * y - wf_round_x_;
            double dy = wf_robot_y_ + sn * x + c * y - wf_round_y_;
            double along = dx * std::cos(wf_round_yaw_) + dy * std::sin(wf_round_yaw_);
            double across = -dx * std::sin(wf_round_yaw_) + dy * std::cos(wf_round_yaw_);
            on_round_wall = along < 0.06 && std::fabs(across) < 0.035;
        }
        if (i >= 45 && i < 105 && r < (i < 75 ? 0.25 : 0.30))
        {
            close_left++;
            if (on_round_wall || (wf_wall_valid_ && wf_wall_distance_ > 0.20 &&
                x >= wf_wall_min_x_ - 0.03 && x <= wf_wall_max_x_ + 0.03 &&
                std::fabs(y * std::cos(wf_wall_angle_) -
                          x * std::sin(wf_wall_angle_) - wf_wall_distance_) < 0.025))
                wall_left++;
        }
        if (i >= 285 && i < 315 && r < 0.25)
        {
            close_right++;
            if (wf_right_valid_ && wf_right_distance_ > 0.20 &&
                x >= wf_right_min_x_ - 0.03 && x <= wf_right_max_x_ + 0.03 &&
                std::fabs(y * std::cos(wf_right_angle_) -
                          x * std::sin(wf_right_angle_) + wf_right_distance_) < 0.025)
                wall_right++;
        }
        if (wf_corridor_valid_)
        {
            double along = x * std::cos(corridor_angle) + y * std::sin(corridor_angle);
            double across = -x * std::sin(corridor_angle) + y * std::cos(corridor_angle);
            if (along > 0.0 && std::fabs(across) < 0.20 && along < wf_corridor_front_)
                wf_corridor_front_ = along;
        }
    }
    wf_left_wall_only_ = close_left > 0 && close_left == wall_left;
    wf_right_wall_only_ = close_right > 0 && close_right == wall_right;

    /****************************************************************
    ** [NEW END] Left-wall fitting and endpoint detection
    ****************************************************************/

}


/********************************************************************************
** Publish velocity
********************************************************************************/

void WallFollower::update_cmd_vel(
    double linear,
    double angular)
{
    geometry_msgs::msg::Twist cmd_vel;

    cmd_vel.linear.x = linear;
    cmd_vel.angular.z = angular;

    cmd_vel_pub_->publish(cmd_vel);
}


/********************************************************************************
** Wall follower logic
********************************************************************************/

bool pl_near;

void WallFollower::update_callback()
{
    /****************************************************************
    ** [MODIFIED V7] State: 0 follow, 1 approach, 2 round,
    ** 3 face a left exit, 4 travel through it.
    ** [NEW V8] 5 finish a thin-wall U-turn without selecting a different wall.
    ** [NEW V9] 6 committed front-obstacle right turn; avoid left/right chatter.
    ** Failed manoeuvres return to follow/search; no latched timeout stop.
    ** Count new laser frames, and use the existing yaw from odometry.
    ****************************************************************/
    static int state = 0, end_count = 0, aligned_count = 0;
    // [NEW V7] Local exit memory; no map and no header additions.
    static int exit_count = 0, exit_aligned = 0;
    static double exit_yaw = 0.0, exit_start_x = 0.0, exit_start_y = 0.0;
    static double exit_cooldown = 0.0;
    static unsigned long previous_scan = 0;
    static double previous_yaw = 0.0, turn_angle = 0.0;
    static double last_end_x = 0.0, last_end_y = 0.0;
    static double turn_elapsed = 0.0, previous_sensor_time = 0.0; // [MODIFIED V6]
    /****************************************************************
    ** [NEW V4] Local controller memory and smooth ordinary commands.
    ** Emergency avoidance below bypasses smoothing and keeps its commands.
    ****************************************************************/
    static double command_linear = 0.0, command_angular = 0.0;
    static double filtered_distance = wf_target_distance_, filtered_angle = 0.0;
    static bool filter_ready = false;
    static double end_world_x = 0.0, end_world_y = 0.0;
    static double approach_distance = wf_target_distance_; // [NEW V9]
    static double round_radius = wf_round_radius_; // [NEW V9]
    static double avoid_turn = 0.0; // [NEW V9] Signed accumulated turn.
    static int avoid_clear_count = 0;
    static double round_entry_yaw = 0.0; // [NEW V8] Incoming wall direction.
    static double settle_start_x = 0.0, settle_start_y = 0.0; // [NEW V8]
    static auto last_update = std::chrono::steady_clock::now();
    static auto last_wall_seen = std::chrono::steady_clock::now();
    auto now = std::chrono::steady_clock::now();
    double dt = std::chrono::duration<double>(now - last_update).count();
    last_update = now;
    if (dt < 0.001) dt = 0.001;
    if (dt > 0.05) dt = 0.05;
    auto clamp = [](double value, double low, double high)
    {
        return value < low ? low : (value > high ? high : value);
    };
    /****************************************************************
    ** [MODIFIED V12] Default command speed limit is 0.40 m/s.
    ** Keep the V11 adaptive braking, corridor and turning speed limits.
    ** Starting parameters require no header or launch-file changes.
    ****************************************************************/
    static const double max_speed = clamp(
        this->declare_parameter<double>("max_linear_speed", 0.40), 0.05, 0.40); // [MODIFIED V12]
    static const double brake_deceleration = clamp(
        this->declare_parameter<double>("brake_deceleration", 0.35), 0.10, 0.60);
    double motion_front = 0.0; // Set from the valid scan before any moving command.
    double scan_age = std::chrono::duration<double>(now - wf_scan_time_).count();
    double odom_age = std::chrono::duration<double>(now - wf_odom_time_).count();
    bool fresh_odom = wf_odom_valid_ && odom_age >= 0.0 && odom_age < 0.40;
    auto smooth_command = [&](double linear, double angular)
    {
        /************************************************************
        ** [NEW V11] Speed envelope for ALL translating manoeuvres.
        ** Assume a conservative deceleration; not a calibrated guarantee.
        ** v*T + v*v/(2*a) must fit before a 0.42 m stopping reserve.
        ** Right avoidance triggers earlier, at 0.45 m; the 3 cm gap
        ** prevents asymptotically stopping just before the turn trigger.
        ** Delay includes scan period, current scan age and motor/network lag.
        ************************************************************/
        double delay = 0.15 + wf_scan_period_ + clamp(scan_age, 0.0, 0.50);
        double free_distance = std::fmax(0.0, motion_front - 0.42);
        double at = brake_deceleration * delay;
        double braking_limit = std::sqrt(at * at +
            2.0 * brake_deceleration * free_distance) - at;
        double speed_limit = std::fmin(max_speed, braking_limit);
        if (!fresh_odom) speed_limit = std::fmin(speed_limit, 0.08);
        /************************************************************
        ** [MODIFIED V13] Faster curves with a curvature-preserving cap.
        ** Clipping only v would tighten an endpoint arc unintentionally.
        ************************************************************/
        double requested_linear = linear;
        if (std::fabs(angular) > 0.01)
        {
            double lateral_limit = state == 2 && linear > 0.001 ?
                std::sqrt(0.18 * linear / std::fabs(angular)) : 0.18 / std::fabs(angular);
            speed_limit = std::fmin(speed_limit, lateral_limit);
        }
        // [NEW V13] Brake near the stored start before the return latch triggers.
        if (wf_return_armed_ && wf_start_distance_ < wf_return_radius_ + 0.45)
            speed_limit = std::fmin(speed_limit, std::fmax(0.08,
                0.10 + 0.60 * (wf_start_distance_ - wf_return_radius_)));
        linear = clamp(linear, 0.0, speed_limit);
        if (state == 2 && requested_linear > 0.001 && linear < requested_linear)
            angular *= linear / requested_linear;
        command_linear = linear < command_linear ? linear :
            clamp(linear, command_linear - 0.45 * dt, command_linear + 0.45 * dt);
        // [MODIFIED V13] Faster response/removal of residual turn commands.
        command_angular = clamp(angular, command_angular - 2.20 * dt,
                                        command_angular + 2.20 * dt);
        update_cmd_vel(command_linear, command_angular);
    };
    // [NEW V8] Never leave a wall-selection lock after a cancelled manoeuvre.
    if (state != 2 && state != 5) wf_round_lock_ = false;
    bool new_scan = (wf_scan_sequence_ != previous_scan);
    previous_scan = wf_scan_sequence_;
    /****************************************************************
    ** [NEW V6] Count only advancing scan timestamps, not paused wall time.
    ** Ignore clock resets and long missing-data jumps.
    ****************************************************************/
    if (new_scan)
    {
        double scan_dt = wf_sensor_time_ - previous_sensor_time;
        if (scan_dt > 0.0 && scan_dt < 1.0)
        {
            if (state != 0) turn_elapsed += scan_dt;
            exit_cooldown = clamp(exit_cooldown - scan_dt, 0.0, 10.0);
        }
        previous_sensor_time = wf_sensor_time_;
    }
    double yaw_change = robot_pose_ - previous_yaw;
    previous_yaw = robot_pose_;
    if (state == 2 || state == 5)
        turn_angle += std::atan2(std::sin(yaw_change), std::cos(yaw_change));
    if (state == 6) // [NEW V9] Right turns accumulate positive progress.
        avoid_turn -= std::atan2(std::sin(yaw_change), std::cos(yaw_change));

    /****************************************************************
    ** Stop after returning near starting point
    ****************************************************************/

    if (near_start)
    {
        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
            "stop=returned_to_start"); // [NEW V6]
        command_linear = command_angular = 0.0; // [NEW V4] Reset command memory.
        update_cmd_vel(0.0, 0.0);
        return;
    }


    /****************************************************************
    ** Read important laser directions
    ****************************************************************/

    double front = scan_data_[FRONT];
    double left_front = scan_data_[LEFT_FRONT];
    double front_left = scan_data_[FRONT_LEFT];
    double front_right = scan_data_[FRONT_RIGHT];


    /****************************************************************
    ** Print sensor values for debugging
    ****************************************************************/

    RCLCPP_INFO_THROTTLE(
        this->get_logger(),
        *this->get_clock(),
        500,
        "front=%.2f left_front=%.2f front_left=%.2f front_right=%.2f state=%d wall=%d turn=%.2f elapsed=%.1f", // [MODIFIED V6]
        front,
        left_front,
        front_left,
        front_right, state, int(wf_wall_valid_), turn_angle, turn_elapsed);


    /****************************************************************
    ** Wait until valid laser data arrives
    ****************************************************************/

    if (
        front <= 0.0 || left_front <= 0.0 || !wf_scan_valid_ ||
        std::chrono::duration<double>(std::chrono::steady_clock::now() -
                                     wf_scan_time_).count() > 0.5) // [MODIFIED] Scan readiness
    {
        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
            "stop=scan_invalid_or_stale"); // [NEW V6] Resumes when data returns.
        command_linear = command_angular = 0.0; // [NEW V4] Reset command memory.
        update_cmd_vel(0.0, 0.0);
        return;
    }


    /****************************************************************
    ** [NEW V7] A confirmed left exit has priority over a normal right
    ** corner. Latch its heading so the front wall cannot replace it.
    ** Two different scans confirm entry and completion of the turn.
    ****************************************************************/
    /****************************************************************
    ** [NEW V9] An angled side wall is not a frontal obstacle when a
    ** valid corridor provides a clear path in its own direction.
    ** Cylinders and end walls remain in the projected front clearance.
    ****************************************************************/
    front = wf_corridor_valid_ && (state == 0 || state == 1 || state == 5) ?
        wf_corridor_front_ : (front < wf_forward_clearance_ ? front : wf_forward_clearance_);
    motion_front = front; // [NEW V11] Front envelope also covers exit/endpoint travel.
    const double target_distance = wf_corridor_valid_ ?
        std::fmin(wf_target_distance_, 0.5 * (wf_wall_distance_ + wf_right_distance_)) :
        wf_target_distance_;
    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
        "corridor=%d left_d=%.2f right_d=%.2f target=%.2f path_front=%.2f cmd_v=%.2f odom_v=%.2f vmax=%.2f", // [MODIFIED V11]
        int(wf_corridor_valid_), wf_wall_distance_, wf_right_distance_, target_distance, front,
        command_linear, wf_measured_speed_, max_speed);
    if (state == 0 && new_scan)
    {
        bool exit_detected = wf_odom_valid_ && exit_cooldown <= 0.0 &&
            front < 0.65 && wf_left_exit_ &&
            (left_front > 0.65 || !wf_wall_valid_ || wf_wall_end_visible_);
        exit_count = exit_detected ? exit_count + 1 : 0;
        if (exit_count >= 2)
        {
            state = 3;
            exit_yaw = robot_pose_ + wf_exit_angle_;
            turn_elapsed = 0.0;
            exit_count = exit_aligned = end_count = aligned_count = 0;
            filter_ready = false;
            RCLCPP_INFO(this->get_logger(), "manoeuvre=left_exit");
        }
        else if (exit_detected)
        {
            // Wait for the second scan without starting the unwanted right turn.
            command_linear = command_angular = 0.0;
            update_cmd_vel(0.0, 0.0);
            return;
        }
    }
    // [NEW V7] Keep the confirmation hold between laser callbacks too.
    if (state == 0 && exit_count > 0)
    {
        command_linear = command_angular = 0.0;
        update_cmd_vel(0.0, 0.0);
        return;
    }
    if (state == 3 || state == 4)
    {
        double error = exit_yaw - robot_pose_;
        error = std::atan2(std::sin(error), std::cos(error));
        if (state == 3 && new_scan)
            exit_aligned = std::fabs(error) < 0.10 ? exit_aligned + 1 : 0;
        if (state == 3 && exit_aligned >= 2)
        {
            state = 4;
            exit_start_x = wf_robot_x_; exit_start_y = wf_robot_y_;
            turn_elapsed = 0.0;
        }
        bool completed = state == 4 &&
            std::hypot(wf_robot_x_ - exit_start_x, wf_robot_y_ - exit_start_y) > 0.60;
        bool blocked = state == 4 && front < 0.46; // [MODIFIED V11] Match the front braking envelope.
        if (completed || blocked || turn_elapsed > 12.0)
        {
            state = 0;
            filter_ready = false;
            exit_cooldown = 3.0;
            exit_aligned = 0;
            RCLCPP_INFO(this->get_logger(), "recover=exit_finished_or_blocked");
        }
        else
        {
            // Turn without forward motion; fitting is suspended during entry.
            double linear = state == 3 ? 0.0 : 0.24; // [MODIFIED V13] Faster clear exit transit.
            double angular = clamp(2.00 * error, -0.90, 0.90); // [MODIFIED V13] Exit turn cap, rad/s.
            smooth_command(linear, angular);
            return;
        }
    }

    /****************************************************************
    ** [NEW V9] Keep one avoidance direction until a forward passage is
    ** clear on two new scans. Side sectors cannot reverse it mid-turn.
    ** A timeout returns to ordinary control, never a permanent stop.
    ****************************************************************/
    if (state == 6)
    {
        bool clear = front > 0.55 && avoid_turn > 0.45 &&
            (left_front > 0.23 || wf_left_wall_only_) &&
            (front_right > 0.23 || wf_right_wall_only_);
        if (new_scan) avoid_clear_count = clear ? avoid_clear_count + 1 : 0;
        if (avoid_clear_count >= 2 || turn_elapsed > 15.0 || avoid_turn > 3.65)
        {
            state = 0;
            filter_ready = false;
            avoid_clear_count = 0;
        }
        else
        {
            // [MODIFIED V13] Turn briskly while blocked, then slow for confirmation.
            command_linear = 0.0; command_angular = front > 0.55 ? -0.40 : -0.85;
            update_cmd_vel(0.0, command_angular);
            return;
        }
    }

    /****************************************************************
    ** 1. Obstacle directly in front
    **
    ** Stop moving forward and turn right.
    ****************************************************************/

    if (front < 0.45)
    {
        /************************************************************
        ** [NEW V8] Pause translation when another wall blocks the arc.
        ** Retain the endpoint; do not turn right and adopt that wall.
        ** Original right avoidance remains for a close left-side object.
        ************************************************************/
        if (state == 2 && turn_angle < 3.00 &&
            left_front >= 0.30 && front_left >= 0.25 && turn_elapsed <= 45.0)
        {
            command_linear = 0.0;
            command_angular = clamp(1.2 * (3.141592653589793 - turn_angle), 0.12, 0.35);
            update_cmd_vel(0.0, command_angular);
            return;
        }
        // [MODIFIED V9] Latch right avoidance; side walls cannot flip its direction.
        wf_round_lock_ = false;
        state = 6;
        avoid_turn = 0.0;
        avoid_clear_count = 0;
        turn_elapsed = 0.0;
        end_count = aligned_count = 0;
        command_linear = 0.0; command_angular = -0.85; // [MODIFIED V13] Faster front-obstacle rotation.
        filter_ready = false;
        update_cmd_vel(
            0.0,
            -0.85); // [MODIFIED V13]

        return;
    }


    /****************************************************************
    ** [MODIFIED] Original side protection moved before endpoint turns
    ** [MODIFIED V7] Keep thresholds; rotate without driving into cylinders.
    ****************************************************************/
    if ((left_front < 0.30 || front_left < 0.25) && !wf_left_wall_only_) // [MODIFIED V9]
    {
        state = 0;
        wf_round_lock_ = false; // [NEW V8]
        end_count = aligned_count = 0;
        command_linear = 0.0; command_angular = -0.60; // [MODIFIED V13] Side avoidance remains in place.
        filter_ready = false;
        update_cmd_vel(0.0, -0.60); // [MODIFIED V13]
        return;
    }

    if (front_right < 0.25 && !wf_right_wall_only_) // [MODIFIED V9]
    {
        state = 0;
        wf_round_lock_ = false; // [NEW V8]
        end_count = aligned_count = 0;
        command_linear = 0.0; command_angular = 0.50; // [MODIFIED V13]
        filter_ready = false;
        update_cmd_vel(0.0, 0.50); // [MODIFIED V13]
        return;
    }

    /****************************************************************
    ** [MODIFIED V4] Confirm a wall endpoint with odometry available
    ** Short-term agreement is checked over three different scans.
    ****************************************************************/
    if (state == 0 && new_scan)
    {
        bool end_detected = wf_odom_valid_ && wf_wall_valid_ && wf_wall_end_visible_ &&
            wf_wall_min_x_ < -0.05 && wf_wall_end_x_ >= -0.10 &&
            wf_wall_end_x_ < 0.65 && wf_wall_end_y_ > 0.20 &&
            std::fabs(wf_wall_angle_) < 0.35 && front > 0.55;

        if (end_detected)
        {
            if (end_count > 0 &&
                std::hypot(wf_wall_end_x_ - last_end_x,
                           wf_wall_end_y_ - last_end_y) > 0.12)
                end_count = 0;
            last_end_x = wf_wall_end_x_;
            last_end_y = wf_wall_end_y_;
            if (++end_count >= 3)
            {
                state = 1;
                end_count = 0;
                turn_elapsed = 0.0; // [MODIFIED V6]
                /****************************************************
                ** [NEW V8] Save the tip NOW, while it is visible.
                ** Later occlusion must not fall back to a wide search arc.
                ****************************************************/
                double c = std::cos(robot_pose_), sn = std::sin(robot_pose_);
                end_world_x = wf_robot_x_ + c * wf_wall_end_x_ - sn * wf_wall_end_y_;
                end_world_y = wf_robot_y_ + sn * wf_wall_end_x_ + c * wf_wall_end_y_;
                round_entry_yaw = robot_pose_ + wf_wall_angle_;
                // [NEW V9] Keep the feasible narrow-corridor distance on approach.
                approach_distance = target_distance;
                round_radius = wf_corridor_valid_ ?
                    clamp(target_distance, 0.30, wf_round_radius_) : wf_round_radius_;
                RCLCPP_INFO(this->get_logger(), "manoeuvre=endpoint_approach");
            }
        }
        else end_count = 0;
    }

    /****************************************************************
    ** [MODIFIED V4] Approach the end, then follow a controlled left arc.
    ** Remember ONE endpoint in odometry coordinates during this manoeuvre.
    ** [MODIFIED V8] Keep the original endpoint/line; use a tighter 0.38 m arc.
    ****************************************************************/
    if (state == 1 || state == 2)
    {
        /************************************************************
        ** [MODIFIED V6] Timeout abandons this arc and resumes wall search.
        ** Do not permanently park the robot because one turn failed.
        ************************************************************/
        if (turn_elapsed > 45.0)
        {
            state = 0;
            wf_round_lock_ = false; // [NEW V8]
            end_count = aligned_count = 0;
            filter_ready = false;
            RCLCPP_INFO(this->get_logger(), "recover=endpoint_timeout");
        }

        if (state == 1)
        {
            /********************************************************
            ** [MODIFIED V8] Approach the stored tip even if the laser
            ** fit temporarily disappears. Pass it by 6 cm before turning.
            ** Follow the incoming line, not a newly detected front wall.
            ********************************************************/
            double dx = end_world_x - wf_robot_x_, dy = end_world_y - wf_robot_y_;
            double c = std::cos(robot_pose_), sn = std::sin(robot_pose_);
            double tip_x = c * dx + sn * dy, tip_y = -sn * dx + c * dy;
            double heading = round_entry_yaw - robot_pose_;
            heading = std::atan2(std::sin(heading), std::cos(heading));
            if (tip_x > -0.06)
            {
                double distance = tip_y * std::cos(heading) - tip_x * std::sin(heading);
                double angular = clamp(1.10 * (distance - approach_distance) +
                                       1.60 * heading, -0.30, 0.30);
                smooth_command(0.22, angular); // [MODIFIED V13] Faster endpoint approach.
                return;
            }
            state = 2;
            turn_angle = 0.0;
            aligned_count = 0;
            wf_round_x_ = end_world_x; wf_round_y_ = end_world_y;
            wf_round_yaw_ = round_entry_yaw;
            wf_round_lock_ = true;
            RCLCPP_INFO(this->get_logger(), "manoeuvre=endpoint_round");
        }

        if (state == 2 && new_scan)
        {
            // [MODIFIED V8] A thin-wall end needs almost a U-turn, not 43 degrees.
            double heading_error = round_entry_yaw + 3.141592653589793 - robot_pose_;
            heading_error = std::atan2(std::sin(heading_error), std::cos(heading_error));
            bool aligned = turn_angle > 2.75 && std::fabs(heading_error) < 0.15 &&
                wf_round_lock_ && wf_wall_valid_ && wf_wall_min_x_ < 0.15 && wf_wall_max_x_ > 0.25 &&
                std::fabs(wf_wall_angle_) < 0.15 &&
                std::fabs(wf_wall_distance_ - wf_target_distance_) < 0.10 && front > 0.55;
            aligned_count = aligned ? aligned_count + 1 : 0;
            if (aligned_count >= 2) // [MODIFIED V4] Avoid unnecessary extra rotation.
            {
                state = 0;
                wf_round_lock_ = false; // [NEW V8] Original wall acquired on its other face.
                aligned_count = 0;
                filter_ready = false; // [NEW V4] Initialise the newly acquired wall.
                RCLCPP_INFO(this->get_logger(), "manoeuvre=endpoint_complete");
            }
        }

        /************************************************************
        ** [MODIFIED V6] Excess angle exits the manoeuvre, not the run.
        ** Restore ordinary wall following/search without a zero command.
        ************************************************************/
        if (state == 2 && turn_angle > 3.80)
        {
            state = 0;
            wf_round_lock_ = false; // [NEW V8]
            end_count = aligned_count = 0;
            filter_ready = false;
            RCLCPP_INFO(this->get_logger(), "recover=excessive_turn");
        }

        /************************************************************
        ** [NEW V8] End rotation near the original wall's reverse heading.
        ** If its fit is missing, creep straight briefly with the lock held.
        ************************************************************/
        double finish_error = round_entry_yaw + 3.141592653589793 - robot_pose_;
        finish_error = std::atan2(std::sin(finish_error), std::cos(finish_error));
        if (state == 2 && turn_angle > 3.02 && std::fabs(finish_error) < 0.10)
        {
            state = 5;
            settle_start_x = wf_robot_x_; settle_start_y = wf_robot_y_;
            turn_elapsed = 0.0;
            filter_ready = false;
        }

        if (state == 2)
        {
            /************************************************************
            ** [MODIFIED V8] Tighter arc with radius AND tangent feedback.
            ** Too far: bend inward; too close: reduce inward curvature.
            ** Excess rotation is handled by recover=excessive_turn above.
            ************************************************************/
            /************************************************************
            ** [NEW V6] Once alignment is seen, use wall feedback while
            ** confirming it; avoid extra arc rotation over another scan.
            ************************************************************/
            if (aligned_count > 0 && wf_wall_valid_)
            {
                double angular = clamp(1.10 * (wf_wall_distance_ - wf_target_distance_) +
                                       1.60 * wf_wall_angle_, -0.35, 0.35);
                smooth_command(0.18, angular); // [MODIFIED V13] Wall alignment confirmation.
                return;
            }
            double dx = end_world_x - wf_robot_x_;
            double dy = end_world_y - wf_robot_y_;
            double c = std::cos(robot_pose_), sn = std::sin(robot_pose_);
            double end_x = c * dx + sn * dy;
            double end_y = -sn * dx + c * dy;
            double radius = std::hypot(end_x, end_y);
            double tangent_error = std::atan2(end_y, end_x) - 3.141592653589793 / 2.0;
            tangent_error = std::atan2(std::sin(tangent_error), std::cos(tangent_error));
            double linear = 0.26; // [MODIFIED V13] Clear-space endpoint arc.
            if (front < 0.75 || radius < 0.32 || std::fabs(tangent_error) > 0.30)
                linear = 0.20; // [MODIFIED V13] Restricted arc still obeys braking/curvature cap.
            double angular = linear / round_radius +
                0.90 * (radius - round_radius) + 1.20 * tangent_error;
            // Never force a tight left turn when already too close to the tip.
            angular = clamp(angular, -0.15, 0.75); // [MODIFIED V11]
            // [NEW V8] Brake rotation before the desired reverse heading.
            if (turn_angle > 2.60)
            {
                linear = 0.14; // [MODIFIED V13] Reduce speed before final heading.
                angular = clamp(2.00 * finish_error, -0.25, 0.70);
            }
            smooth_command(linear, angular);
            return;
        }
    }

    /****************************************************************
    ** [NEW V8] Short straight reacquisition after a thin-wall U-turn.
    ** Safety checks above still apply. Never latch a permanent stop.
    ****************************************************************/
    if (state == 5)
    {
        double heading = round_entry_yaw + 3.141592653589793 - robot_pose_;
        heading = std::atan2(std::sin(heading), std::cos(heading));
        bool acquired = wf_wall_valid_ && wf_wall_min_x_ < 0.15 &&
            wf_wall_max_x_ > 0.25 && std::fabs(wf_wall_angle_) < 0.15;
        if (new_scan) aligned_count = acquired ? aligned_count + 1 : 0;
        bool expired = turn_elapsed > 4.0 ||
            std::hypot(wf_robot_x_ - settle_start_x, wf_robot_y_ - settle_start_y) > 0.30;
        if (aligned_count >= 2 || expired)
        {
            state = 0;
            wf_round_lock_ = false;
            filter_ready = false;
            aligned_count = end_count = 0;
            RCLCPP_INFO(this->get_logger(), expired ?
                "recover=endpoint_reacquire_timeout" : "manoeuvre=endpoint_complete");
        }
        else
        {
            smooth_command(0.16, clamp(1.70 * heading, -0.30, 0.30)); // [MODIFIED V13]
            return;
        }
    }

    /****************************************************************
    ** [MODIFIED V4] Smooth wall distance and heading once per laser frame.
    ** Speed up on aligned straight walls; slow for errors and front objects.
    ** Do not treat an oblique sector reading as perpendicular wall distance.
    ****************************************************************/
    double angular = 0.0, linear = 0.08;
    if (wf_wall_valid_)
    {
        if (!filter_ready)
        {
            filtered_distance = wf_wall_distance_;
            filtered_angle = wf_wall_angle_;
            filter_ready = true;
        }
        else if (new_scan)
        {
            filtered_distance += 0.65 * (wf_wall_distance_ - filtered_distance);
            filtered_angle += 0.70 * (wf_wall_angle_ - filtered_angle);
        }
        last_wall_seen = now;
        /************************************************************
        ** [MODIFIED V11] Speed-dependent lookahead corrects wall drift
        ** before it becomes a large lateral error at higher speed.
        ************************************************************/
        double error = filtered_distance - target_distance;
        double lookahead = 0.12 + 0.80 * std::fmax(command_linear, wf_measured_speed_);
        double predicted_error = error + lookahead * std::sin(filtered_angle);
        angular = clamp(1.40 * predicted_error + 1.80 * filtered_angle, -0.55, 0.55);
        linear = max_speed / (1.0 + 3.0 * std::fabs(predicted_error) +
                                   2.0 * std::fabs(filtered_angle));
        linear = clamp(linear, 0.08, max_speed);
        if (state == 1 || end_count > 0)
            linear = std::fmin(linear, 0.22); // [MODIFIED V13] Faster pending-end confirmation.

    }
    else
    {
        /************************************************************
        ** [MODIFIED V4] Brief loss: decay steering; prolonged loss:
        ** search left gently. Original obstacle protection stays first.
        ************************************************************/
        double lost_time = std::chrono::duration<double>(now - last_wall_seen).count();
        if (filter_ready && lost_time < 0.60)
            angular = command_angular * std::exp(-2.0 * dt);
        else
        {
            filter_ready = false;
            angular = 0.28; // [MODIFIED V13] Faster search with a bounded arc radius.
        }
        linear = 0.16; // [MODIFIED V13] Valid scan required; front braking still applies.
    }

    /****************************************************************
    ** [NEW V9] In a narrow corridor, steer toward feasible spacing and
    ** align with both walls. Cap speed/steering to reduce weaving.
    ****************************************************************/
    if (wf_corridor_valid_ && state == 0)
    {
        // [MODIFIED V11] Increase corridor speed only when side margins allow it.
        double heading = 0.5 * (wf_wall_angle_ + wf_right_angle_);
        double lookahead = 0.12 + 0.80 * std::fmax(command_linear, wf_measured_speed_);
        double error = wf_wall_distance_ - target_distance;
        double predicted_error = error + lookahead * std::sin(heading);
        angular = clamp(1.50 * predicted_error + 1.90 * heading, -0.45, 0.45);
        double side_margin = std::fmin(wf_wall_distance_, wf_right_distance_) - 0.20;
        double corridor_speed = clamp(0.14 + side_margin, 0.10, 0.92 * max_speed);
        linear = corridor_speed / (1.0 + 4.0 * std::fabs(heading) +
                                        4.0 * std::fabs(predicted_error));
        if (end_count > 0) linear = std::fmin(linear, 0.22); // [MODIFIED V13] Faster pending-end confirmation.
    }

    // [MODIFIED V11] Distance/delay braking in smooth_command replaces the
    // old fixed 0.22 m/s front cap, including on endpoint/exit manoeuvres.
    smooth_command(linear, angular);

}


/*******************************************************************************
** Main
*******************************************************************************/

int main(
    int argc,
    char ** argv)
{
    rclcpp::init(argc, argv);

    rclcpp::spin(
        std::make_shared<WallFollower>());

    rclcpp::shutdown();

    return 0;
}