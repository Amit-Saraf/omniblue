#include <iostream>
#include <cmath>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>

#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <vector>
#include <Eigen/Dense>
#include <geometry_msgs/msg/pose2_d.hpp>

using std::placeholders::_1;

struct RobotState
{
    double x = 0.0;
    double y = 0.0;

    double theta = 0.0;

    double vx = 0.0;
    double vy = 0.0;

    double wz = 0.0;
};

class FormationSMC : public rclcpp::Node
{
public:

    FormationSMC()
    : Node("formation_smc_controller")
    {
        std::string ns = this->get_namespace();

        if(ns == "/omniblue1")
            robot_id_ = 1;
        else if(ns == "/omniblue2")
            robot_id_ = 2;
        else if(ns == "/omniblue3")
            robot_id_ = 3;
        else
            robot_id_ = 1;

        RCLCPP_INFO(
            get_logger(),
            "Robot ID = %d",
            robot_id_);

        // equilateral triangle, side 3.0 m, centroid at origin
        r1_ << -1.5, -0.866;
        r2_ <<  1.5, -0.866;
        r3_ <<  0.0,  1.732;


        goal_theta_ = 0.0;

        odom1_sub_ =
            create_subscription<nav_msgs::msg::Odometry>(
                "/omniblue1/odom",
                10,
                std::bind(
                    &FormationSMC::odom1Callback,
                    this,
                    _1));

        odom2_sub_ =
            create_subscription<nav_msgs::msg::Odometry>(
                "/omniblue2/odom",
                10,
                std::bind(
                    &FormationSMC::odom2Callback,
                    this,
                    _1));

        odom3_sub_ =
            create_subscription<nav_msgs::msg::Odometry>(
                "/omniblue3/odom",
                10,
                std::bind(
                    &FormationSMC::odom3Callback,
                    this,
                    _1));

        
        goal_sub_ =
            create_subscription<geometry_msgs::msg::Pose2D>(
                "/formation_goal",
                10,
                std::bind(
                    &FormationSMC::goalCallback,
                    this,
                    _1));

        scan_sub_ =
            create_subscription<sensor_msgs::msg::LaserScan>(
                "scan",
                10,
                std::bind(
                    &FormationSMC::scanCallback,
                    this,
                    _1));

        cmd_pub_ =
            create_publisher<geometry_msgs::msg::Twist>(
                "cmd_vel",
                10);

        timer_ =
            create_wall_timer(
                std::chrono::milliseconds(16),
                std::bind(
                    &FormationSMC::controlLoop,
                    this));
    }

private:

    int robot_id_;

    RobotState omniblue1_;
    RobotState omniblue2_;
    RobotState omniblue3_;

    bool robot1_received_ = false;
    bool robot2_received_ = false;
    bool robot3_received_ = false;
    bool goal_received_ = false;

    Eigen::Vector2d r1_;
    Eigen::Vector2d r2_;
    Eigen::Vector2d r3_;

    double goal_x_ = 0.0;
    double goal_y_ = 0.0;
    double goal_theta_ = 0.0;

    double obstacle_distance_ = 100.0;
    double obstacle_angle_ = 0.0;

    // discrete obstacle cluster representatives (world frame)
    std::vector<double> obs_px_;
    std::vector<double> obs_py_;
    std::vector<double> obs_r_;

    double prev_fx_ = 0.0;
    double prev_fy_ = 0.0;

    double ux_ = 0.0;
    double uy_ = 0.0;
    double uw_ = 0.0;

    double dt_ = 0.016;

    // virtual formation center (virtual structure)
    double pc_x_ = 0.0;
    double pc_y_ = 0.0;
    double pc_theta_ = 0.0;
    bool center_init_ = false;

    // slot offset assigned by proximity at init (replaces fixed id->slot map)
    Eigen::Vector2d r_assigned_;

    // assigned offsets of ALL robots (index = robot 1..3), so every
    // controller can compute fleet-wide slot errors identically
    Eigen::Vector2d r_all_[3];

    // filtered avoidance force (for smooth f_dot)
    double f_filt_x_ = 0.0;
    double f_filt_y_ = 0.0;

    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom1_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom2_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom3_sub_;

    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;

    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;

    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Subscription<geometry_msgs::msg::Pose2D>::SharedPtr goal_sub_;
    void updateRobotState(
        RobotState & robot,
        const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        robot.x =
            msg->pose.pose.position.x;

        robot.y =
            msg->pose.pose.position.y;

        robot.vx =
            msg->twist.twist.linear.x;

        robot.vy =
            msg->twist.twist.linear.y;

        robot.wz =
            msg->twist.twist.angular.z;

        tf2::Quaternion q(
            msg->pose.pose.orientation.x,
            msg->pose.pose.orientation.y,
            msg->pose.pose.orientation.z,
            msg->pose.pose.orientation.w);

        tf2::Matrix3x3 m(q);

        double roll, pitch;

        m.getRPY(
            roll,
            pitch,
            robot.theta);
    }

    void odom1Callback(
        const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        updateRobotState(
            omniblue1_,
            msg);

        robot1_received_ = true;
    }

    void odom2Callback(
        const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        updateRobotState(
            omniblue2_,
            msg);

        robot2_received_ = true;
    }

    void odom3Callback(
        const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        updateRobotState(
            omniblue3_,
            msg);

        robot3_received_ = true;
    }

    

    void goalCallback(
        const geometry_msgs::msg::Pose2D::SharedPtr msg)
    {
        goal_x_ = msg->x;
        goal_y_ = msg->y;
        goal_theta_ = msg->theta;
        goal_received_ = true;
    }

    void scanCallback(
        const sensor_msgs::msg::LaserScan::SharedPtr msg)
    {
        obstacle_distance_ = 100.0;
        obstacle_angle_ = 0.0;

        // discrete obstacle clusters, rebuilt each scan
        obs_px_.clear();
        obs_py_.clear();
        obs_r_.clear();

        // cluster-walk state
        bool   have_prev_r = false;
        double prev_r = 0.0;
        bool   cur_valid = false;
        double cur_r = 0.0;
        double cur_px = 0.0;
        double cur_py = 0.0;

        // self pose, to project scan points into world frame
        RobotState self;

        if(robot_id_ == 1)
            self = omniblue1_;
        else if(robot_id_ == 2)
            self = omniblue2_;
        else
            self = omniblue3_;

        const double teammate_radius = 0.45; // reject returns this close to a teammate

        for(size_t i = 0; i < msg->ranges.size(); i++)
        {
            double r = msg->ranges[i];

            if(std::isnan(r) || std::isinf(r))
                continue;

            if(r < msg->range_min || r > msg->range_max)
                continue;

            double angle =
                msg->angle_min +
                i * msg->angle_increment;

            // scan point in world frame
            double px =
                self.x + r * cos(self.theta + angle);

            double py =
                self.y + r * sin(self.theta + angle);

            // reject teammates: skip points near the other two robots
            bool is_teammate = false;

            const RobotState * others[2];

            if(robot_id_ == 1)
            {
                others[0] = &omniblue2_;
                others[1] = &omniblue3_;
            }
            else if(robot_id_ == 2)
            {
                others[0] = &omniblue1_;
                others[1] = &omniblue3_;
            }
            else
            {
                others[0] = &omniblue1_;
                others[1] = &omniblue2_;
            }

            for(int k = 0; k < 2; k++)
            {
                double ddx = px - others[k]->x;
                double ddy = py - others[k]->y;

                if(ddx * ddx + ddy * ddy <
                   teammate_radius * teammate_radius)
                {
                    is_teammate = true;
                    break;
                }
            }

            if(is_teammate)
                continue;

            // ---- cluster into discrete obstacles ----
            // a new cluster starts when the range jumps; within a
            // cluster keep only the nearest point. One representative
            // per obstacle so a robot between two obstacles can sum
            // both forces instead of chattering on the single min.
            const double cluster_gap = 0.5;

            if(have_prev_r &&
               std::abs(r - prev_r) > cluster_gap)
            {
                // close current cluster
                if(cur_valid)
                {
                    obs_px_.push_back(cur_px);
                    obs_py_.push_back(cur_py);
                    obs_r_.push_back(cur_r);
                }
                cur_valid = false;
            }

            if(!cur_valid || r < cur_r)
            {
                cur_r = r;
                cur_px = px;
                cur_py = py;
                cur_valid = true;
            }

            prev_r = r;
            have_prev_r = true;

            if(r < obstacle_distance_)
            {
                obstacle_distance_ = r;
                obstacle_angle_ = angle;
            }
        }

        // flush last cluster
        if(cur_valid)
        {
            obs_px_.push_back(cur_px);
            obs_py_.push_back(cur_py);
            obs_r_.push_back(cur_r);
        }
    }

    double sat(double x, double phi)
    {
        if(x > phi)
            return 1.0;

        if(x < -phi)
            return -1.0;

        return x / phi;
    }
    double wrapAngle(double a)
    {
        while(a > M_PI)
            a -= 2.0 * M_PI;

        while(a < -M_PI)
            a += 2.0 * M_PI;

        return a;
    }

    double saturate(double val, double max_val)
    {
        if(val > max_val)
            return max_val;

        if(val < -max_val)
            return -max_val;

        return val;
    }

    void controlLoop()
    {
        if(!goal_received_)
        {
            return;
        }

        if(!robot1_received_ ||
           !robot2_received_ ||
           !robot3_received_)
        {
            return;
        }

        geometry_msgs::msg::Twist cmd;

        RobotState self;

        if(robot_id_ == 1)
            self = omniblue1_;
        else if(robot_id_ == 2)
            self = omniblue2_;
        else
            self = omniblue3_;



        Eigen::Vector3d x1;

        x1 <<
            self.x,
            self.y,
            self.theta;

        // ---- virtual formation center (virtual structure) ----
        // initialize at the current centroid so robots first form
        // the triangle where they are, then translate rigidly
        if(!center_init_)
        {
            pc_x_ =
                (omniblue1_.x +
                 omniblue2_.x +
                 omniblue3_.x) / 3.0;

            pc_y_ =
                (omniblue1_.y +
                 omniblue2_.y +
                 omniblue3_.y) / 3.0;

            pc_theta_ = goal_theta_;

            // ---- proximity-based slot assignment ----
            // brute force all 6 robot->slot permutations,
            // pick min total squared distance. Deterministic:
            // every robot computes the same assignment.
            Eigen::Matrix2d R0;

            R0 <<
                cos(pc_theta_), -sin(pc_theta_),
                sin(pc_theta_),  cos(pc_theta_);

            Eigen::Vector2d slots[3];

            slots[0] = Eigen::Vector2d(pc_x_, pc_y_) + R0 * r1_;
            slots[1] = Eigen::Vector2d(pc_x_, pc_y_) + R0 * r2_;
            slots[2] = Eigen::Vector2d(pc_x_, pc_y_) + R0 * r3_;

            Eigen::Vector2d robots[3];

            robots[0] = Eigen::Vector2d(omniblue1_.x, omniblue1_.y);
            robots[1] = Eigen::Vector2d(omniblue2_.x, omniblue2_.y);
            robots[2] = Eigen::Vector2d(omniblue3_.x, omniblue3_.y);

            int perms[6][3] = {
                {0,1,2}, {0,2,1},
                {1,0,2}, {1,2,0},
                {2,0,1}, {2,1,0}
            };

            double best_cost = 1e18;
            int best = 0;

            for(int p = 0; p < 6; p++)
            {
                double cost = 0.0;

                for(int k = 0; k < 3; k++)
                {
                    cost +=
                        (robots[k] - slots[perms[p][k]])
                        .squaredNorm();
                }

                if(cost < best_cost)
                {
                    best_cost = cost;
                    best = p;
                }
            }

            for(int k = 0; k < 3; k++)
            {
                int slot_k = perms[best][k];

                if(slot_k == 0)
                    r_all_[k] = r1_;
                else if(slot_k == 1)
                    r_all_[k] = r2_;
                else
                    r_all_[k] = r3_;
            }

            r_assigned_ = r_all_[robot_id_ - 1];

            RCLCPP_INFO(
                get_logger(),
                "R%d assigned slot %d offset=(%.2f %.2f)",
                robot_id_,
                perms[best][robot_id_ - 1] + 1,
                r_assigned_(0),
                r_assigned_(1));

            center_init_ = true;
        }

        // move center toward goal at limited speed
        double vmax_c = 0.3;   // formation cruise speed [m/s]
        double wmax_c = 0.3;   // formation rotation rate [rad/s]

        // formation waits: if any robot is far from its slot
        // (e.g. breaking formation to avoid an obstacle), slow the
        // center so the fleet doesn't run away from it. Identical
        // computation on every robot -> centers stay in sync.
        Eigen::Matrix2d Rc;

        Rc <<
            cos(pc_theta_), -sin(pc_theta_),
            sin(pc_theta_),  cos(pc_theta_);

        const RobotState * all_robots[3] =
            { &omniblue1_, &omniblue2_, &omniblue3_ };

        double max_slot_err = 0.0;

        for(int k = 0; k < 3; k++)
        {
            Eigen::Vector2d slot_k =
                Eigen::Vector2d(pc_x_, pc_y_) +
                Rc * r_all_[k];

            double ex = all_robots[k]->x - slot_k(0);
            double ey = all_robots[k]->y - slot_k(1);

            double err = sqrt(ex * ex + ey * ey);

            if(err > max_slot_err)
                max_slot_err = err;
        }

        double wait_factor =
            1.0 / (1.0 + 2.0 * std::max(0.0, max_slot_err - 0.5));

        double cdx = goal_x_ - pc_x_;
        double cdy = goal_y_ - pc_y_;
        double cdist = sqrt(cdx * cdx + cdy * cdy);

        Eigen::Vector2d vc;
        vc.setZero();

        if(cdist > 1e-3)
        {
            double speed =
                std::min(vmax_c, 1.0 * cdist) *
                wait_factor;

            vc <<
                speed * cdx / cdist,
                speed * cdy / cdist;

            pc_x_ += vc(0) * dt_;
            pc_y_ += vc(1) * dt_;
        }

        double wc =
            saturate(
                1.0 * wrapAngle(goal_theta_ - pc_theta_),
                wmax_c);

        pc_theta_ =
            wrapAngle(pc_theta_ + wc * dt_);

        // desired slot = center + rotated offset
        Eigen::Matrix2d Rf;

        Rf <<
            cos(pc_theta_), -sin(pc_theta_),
            sin(pc_theta_),  cos(pc_theta_);

        Eigen::Vector2d r_rot;

        r_rot = Rf * r_assigned_;

        Eigen::Vector3d xd;

        xd <<
            pc_x_ + r_rot(0),
            pc_y_ + r_rot(1),
            wrapAngle(pc_theta_);
        
        Eigen::Vector3d x2;

        x2 <<
            self.vx,
            self.vy,
            self.wz;

        Eigen::Vector3d x2_world;

        x2_world <<
            cos(self.theta)*self.vx - sin(self.theta)*self.vy,
            sin(self.theta)*self.vx + cos(self.theta)*self.vy,
            self.wz;

        RCLCPP_INFO(
            get_logger(),
            "R%d body=(%.2f %.2f) world=(%.2f %.2f)",
            robot_id_,
            self.vx,
            self.vy,
            x2_world(0),
            x2_world(1));

        RCLCPP_INFO(
            get_logger(),
            "R%d theta=%.2f",
            robot_id_,
            self.theta);

        // slot velocity feedforward: center translation + rotation
        Eigen::Vector3d xd_dot;

        xd_dot <<
            vc(0) - wc * r_rot(1),
            vc(1) + wc * r_rot(0),
            wc;

        // ==== break-formation avoidance (multi-obstacle) ====
        // Sum the solo-style rotated-repulsion force over ALL discrete
        // obstacle clusters. A robot between two obstacles gets one
        // coherent resultant instead of chattering on the single
        // nearest point. The formation spring is blended down by lambda
        // (driven by the NEAREST obstacle) so avoidance is never
        // overpowered; away from obstacles lambda -> 0 and the moving
        // slot pulls the robot back = automatic rejoin.

        double ko_obs = 13.0;   // strengthened
        double sigma  = 0.7;
        double d_infl = 2.0;
        double alpha  = 2.5;    // fixed detour sense (proven solo)

        Eigen::Matrix2d Ra;

        Ra <<
            cos(alpha), -sin(alpha),
            sin(alpha),  cos(alpha);

        Eigen::Vector2d f_obs;
        f_obs.setZero();

        double d_near = 100.0;  // nearest cluster, for lambda + logging

        for(size_t k = 0; k < obs_r_.size(); k++)
        {
            double odx = self.x - obs_px_[k];
            double ody = self.y - obs_py_[k];

            double od = sqrt(odx * odx + ody * ody);

            if(od < 0.001)
                od = 0.001;

            double omag =
                ko_obs *
                (exp(-od / sigma) - exp(-d_infl / sigma));

            if(omag <= 0.0)
                continue;   // outside this obstacle's influence

            Eigen::Vector2d on;

            on << odx / od, ody / od;

            f_obs += omag * (Ra * on);

            if(od < d_near)
                d_near = od;
        }

        // blend factor from the nearest obstacle
        double near_mag =
            ko_obs *
            (exp(-d_near / sigma) - exp(-d_infl / sigma));

        if(near_mag < 0.0)
            near_mag = 0.0;

        double lambda = near_mag / (near_mag + 1.0);

        RCLCPP_INFO_THROTTLE(
            get_logger(),
            *get_clock(),
            500,
            "R%d obstacles=%zu d_near=%.2f lambda=%.2f f=(%.2f %.2f)",
            robot_id_,
            obs_r_.size(),
            d_near,
            lambda,
            f_obs(0),
            f_obs(1));
        // ==== end avoidance ====

        Eigen::Vector3d e1;

        e1(0) = self.x - xd(0);
        e1(1) = self.y - xd(1);
        e1(2) = wrapAngle(self.theta - xd(2));
        
        Eigen::Vector3d e2;
        e2 = x2_world - xd_dot;

        // ---- inter-robot collision avoidance (odom-based) ----
        // pure repulsion, active only below d_rob; zero at the
        // 2.0 m formation spacing so it never fights the formation
        {
            double k_rob = 7.0;
            double sig_r = 0.3;
            double d_rob = 0.9;

            const RobotState * mates[2];

            if(robot_id_ == 1)
            {
                mates[0] = &omniblue2_;
                mates[1] = &omniblue3_;
            }
            else if(robot_id_ == 2)
            {
                mates[0] = &omniblue1_;
                mates[1] = &omniblue3_;
            }
            else
            {
                mates[0] = &omniblue1_;
                mates[1] = &omniblue2_;
            }

            for(int k = 0; k < 2; k++)
            {
                double rdx = self.x - mates[k]->x;
                double rdy = self.y - mates[k]->y;

                double rd =
                    sqrt(rdx * rdx + rdy * rdy);

                if(rd < 0.001)
                    rd = 0.001;

                double rmag =
                    k_rob *
                    (exp(-rd / sig_r) -
                     exp(-d_rob / sig_r));

                if(rmag > 0.0)
                {
                    f_obs(0) += rmag * rdx / rd;
                    f_obs(1) += rmag * rdy / rd;
                }
            }
        }

        // low-pass filter f before differentiating (raw lidar min is noisy)
        double beta = 0.2;

        f_filt_x_ += beta * (f_obs(0) - f_filt_x_);
        f_filt_y_ += beta * (f_obs(1) - f_filt_y_);

        double fx_dot =
            (f_filt_x_ - prev_fx_) / dt_;

        double fy_dot =
            (f_filt_y_ - prev_fy_) / dt_;

        prev_fx_ = f_filt_x_;
        prev_fy_ = f_filt_y_;

        Eigen::Vector3d f;

        f <<
            f_filt_x_,
            f_filt_y_,
            0.0;

        Eigen::Vector3d f_dot;

        f_dot <<
            fx_dot,
            fy_dot,
            0.0;

        // formation spring, blended DOWN near obstacles: at lambda=1
        // the xy spring is at 15% strength — the robot is free to
        // break formation and follow the avoidance force. Heading
        // gain untouched.
        double c_xy =
            1.5 * (1.0 - 0.85 * lambda);

        Eigen::Matrix3d C;

        C <<
            c_xy, 0.0,  0.0,
            0.0,  c_xy, 0.0,
            0.0,  0.0,  1.0;
        // double kf = 1.0;

        Eigen::Vector3d s;

        // s =
        //     e2 +
        //     kt * et3 +
        //     kf * h3 +
        //     f;

        s =
            e2 +
            C * e1 +
            f;

        RCLCPP_INFO(
            get_logger(),
            "R%d s=(%.2f %.2f %.2f)",
            robot_id_,
            s(0),
            s(1),
            s(2));

        Eigen::Matrix3d K;

        K <<
            1.0, 0.0, 0.0,
            0.0, 1.0, 0.0,
            0.0, 0.0, 0.7;

        Eigen::Matrix3d g;

        g <<
            cos(self.theta), -sin(self.theta), 0.0,
            sin(self.theta),  cos(self.theta), 0.0,
            0.0,              0.0,             1.0;

        Eigen::Matrix3d g_dot;

        g_dot <<
            -sin(self.theta) * self.wz,
            -cos(self.theta) * self.wz,
            0.0,

             cos(self.theta) * self.wz,
            -sin(self.theta) * self.wz,
            0.0,

            0.0,
            0.0,
            0.0;

        Eigen::Vector3d u;

        u <<
            ux_,
            uy_,
            uw_;

        RCLCPP_INFO(
            get_logger(),
            "R%d u=(%.2f %.2f %.2f)",
            robot_id_,
            u(0),
            u(1),
            u(2));

        Eigen::Vector3d sliding_term;

        for(int i = 0; i < 3; i++)
        {
            sliding_term(i) =
                K(i,i) *
                sqrt(std::abs(s(i))) *
                sat(s(i), 0.1);
        }
        RCLCPP_INFO(
            get_logger(),
            "R%d slide=(%.2f %.2f %.2f)",
            robot_id_,
            sliding_term(0),
            sliding_term(1),
            sliding_term(2));

        Eigen::Vector3d v;

        v =
            -g.transpose() *
            (
                g_dot * u +
                C * e2 +
                f_dot +
                sliding_term
            );

        // anti-windup, vector form: integrate, then clamp the
        // (ux, uy) NORM so saturation never distorts the direction
        double ux_new = ux_ + v(0) * dt_;
        double uy_new = uy_ + v(1) * dt_;
        double uw_new = uw_ + v(2) * dt_;

        double v_lin_max = 0.8;

        double u_norm =
            sqrt(ux_new * ux_new + uy_new * uy_new);

        if(u_norm > v_lin_max)
        {
            double scale = v_lin_max / u_norm;

            ux_new *= scale;
            uy_new *= scale;
        }

        ux_ = ux_new;
        uy_ = uy_new;

        if(std::abs(uw_new) < 1.0 || uw_new * v(2) < 0.0)
            uw_ = saturate(uw_new, 1.0);

        cmd.linear.x = ux_;
        cmd.linear.y = uy_;
        cmd.angular.z = uw_;
        RCLCPP_INFO(
            get_logger(),
            "R%d u=(%.2f %.2f %.2f)",
            robot_id_,
            ux_,
            uy_,
            uw_);


        cmd_pub_->publish(cmd);


    }
};

int main(int argc, char ** argv)
{

    rclcpp::init(argc, argv);

    rclcpp::spin(
        std::make_shared<FormationSMC>());

    rclcpp::shutdown();

    return 0;
}