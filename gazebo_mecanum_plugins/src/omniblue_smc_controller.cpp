#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>

#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <Eigen/Dense>

using std::placeholders::_1;

class OmniblueSMC : public rclcpp::Node
{
public:

    OmniblueSMC() : Node("omniblue_smc_controller")
    {
        odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
            "/omniblue/odom",
            10,
            std::bind(&OmniblueSMC::odomCallback, this, _1));

        scan_sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
            "/omniblue/scan",
            10,
            std::bind(&OmniblueSMC::scanCallback, this, _1));

        cmd_pub_ = create_publisher<geometry_msgs::msg::Twist>(
            "/omniblue/cmd_vel",
            10);

        timer_ = create_wall_timer(
            std::chrono::milliseconds(16),
            std::bind(&OmniblueSMC::controlLoop, this));

        std::cout << "\nEnter Goal X: ";
        std::cin >> goal_x_;

        std::cout << "Enter Goal Y: ";
        std::cin >> goal_y_;

        std::cout << "Enter Goal Theta(rad): ";
        std::cin >> goal_theta_;
    }

private:

    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;

    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;

    rclcpp::TimerBase::SharedPtr timer_;

    double x_ = 0.0;
    double y_ = 0.0;
    double theta_ = 0.0;

    double vx_world_ = 0.0;
    double vy_world_ = 0.0;
    double wz_ = 0.0;

    double goal_x_ = 0.0;
    double goal_y_ = 0.0;
    double goal_theta_ = 0.0;

    double obstacle_distance_ = 100.0;
    double obstacle_angle_ = 0.0;

    double prev_fx_ = 0.0;
    double prev_fy_ = 0.0;

    double ux_ = 0.0;
    double uy_ = 0.0;
    double uw_ = 0.0;

    double dt_ = 1.0 / 60.0;

    void odomCallback(
        const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        x_ = msg->pose.pose.position.x;
        y_ = msg->pose.pose.position.y;

        vx_world_ = msg->twist.twist.linear.x;
        vy_world_ = msg->twist.twist.linear.y;
        wz_ = msg->twist.twist.angular.z;

        tf2::Quaternion q(
            msg->pose.pose.orientation.x,
            msg->pose.pose.orientation.y,
            msg->pose.pose.orientation.z,
            msg->pose.pose.orientation.w);

        tf2::Matrix3x3 m(q);

        double roll, pitch;

        m.getRPY(roll, pitch, theta_);
    }

    void scanCallback(
        const sensor_msgs::msg::LaserScan::SharedPtr msg)
    {
        obstacle_distance_ = 100.0;
        obstacle_angle_ = 0.0;

        for(size_t i = 0; i < msg->ranges.size(); i++)
        {
            double r = msg->ranges[i];

            if(std::isnan(r) || std::isinf(r))
                continue;

            double angle =
                msg->angle_min +
                i * msg->angle_increment;

            if(r < obstacle_distance_)
            {
                obstacle_distance_ = r;
                obstacle_angle_ = angle;
            }
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

    /*
    Replace sign() with sat() if chattering occurs.

    double sign(double x)
    {
        if(x > 0.0)
            return 1.0;

        if(x < 0.0)
            return -1.0;

        return 0.0;
    }

    

    Then replace:

    sign(s(i))

    with:

    sat(s(i), 0.1)
    */

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
        geometry_msgs::msg::Twist cmd;

        Eigen::Vector3d x1;
        x1 << x_, y_, theta_;

        Eigen::Vector3d xd;
        xd << goal_x_, goal_y_, goal_theta_;

        Eigen::Vector3d x2;
        x2 << vx_world_, vy_world_, wz_;

        Eigen::Vector3d xd_dot;
        xd_dot.setZero();

        Eigen::Vector3d e1;
        e1 = x1 - xd;

        Eigen::Vector3d e2;
        e2 = x2 - xd_dot;

        double obs_x =
            x_ +
            obstacle_distance_ *
            cos(theta_ + obstacle_angle_);

        double obs_y =
            y_ +
            obstacle_distance_ *
            sin(theta_ + obstacle_angle_);

        double dx = x_ - obs_x;
        double dy = y_ - obs_y;

        double d =
            sqrt(dx * dx + dy * dy);

        if(d < 0.001)
            d = 0.001;

        Eigen::Vector2d n;

        n <<
            dx / d,
            dy / d;

        // Small smooth rotation angle
        double alpha = 2.5;

        // Rotation matrix
        Eigen::Matrix2d R;

        R <<
            cos(alpha), -sin(alpha),
            sin(alpha),  cos(alpha);

        // Rotated repulsion direction
        Eigen::Vector2d n_r;

        n_r = R * n;

        // Repulsion gain
        double ko = 7.0;

        // Influence radius
        double sigma = 2;

        Eigen::Vector2d f_obs;

        f_obs =
            ko *
            exp(-d / sigma) *
            n_r;

        double fx_dot =
            (f_obs(0) - prev_fx_) / dt_;

        double fy_dot =
            (f_obs(1) - prev_fy_) / dt_;

        prev_fx_ = f_obs(0);
        prev_fy_ = f_obs(1);

        Eigen::Vector3d f;

        f <<
            f_obs(0),
            f_obs(1),
            0.0;

        Eigen::Vector3d f_dot;

        f_dot <<
            fx_dot,
            fy_dot,
            0.0;

        Eigen::Matrix3d C;

        C <<
            1.5, 0.0, 0.0,
            0.0, 1.5, 0.0,
            0.0, 0.0, 1;

        Eigen::Vector3d s;

        s =
            e2 +
            C * e1 +
            f;

        Eigen::Matrix3d K;

        K <<
            1.0, 0.0, 0.0,
            0.0, 1.0, 0.0,
            0.0, 0.0, 0.7;

        Eigen::Matrix3d g;

        g <<
            cos(theta_), -sin(theta_), 0.0,
            sin(theta_),  cos(theta_), 0.0,
            0.0,           0.0,        1.0;

        Eigen::Matrix3d g_dot;

        g_dot <<
            -sin(theta_) * wz_,
            -cos(theta_) * wz_,
            0.0,

            cos(theta_) * wz_,
            -sin(theta_) * wz_,
            0.0,

            0.0,
            0.0,
            0.0;

        Eigen::Vector3d u;

        u <<
            ux_,
            uy_,
            uw_;

        Eigen::Vector3d sliding_term;

        for(int i = 0; i < 3; i++)
        {
            sliding_term(i) =
                K(i,i) *
                sqrt(std::abs(s(i))) *
                sat(s(i), 0.1);
        }

        Eigen::Vector3d v;

        v =
            -g.inverse() *
            (
                g_dot * u +
                C * e2 +
                f_dot +
                sliding_term
            );

        ux_ += v(0) * dt_;
        uy_ += v(1) * dt_;
        uw_ += v(2) * dt_;

        ux_ = saturate(ux_, 0.8);
        uy_ = saturate(uy_, 0.8);
        uw_ = saturate(uw_, 1.0);

        cmd.linear.x = ux_;
        cmd.linear.y = uy_;
        cmd.angular.z = uw_;

        cmd_pub_->publish(cmd);
    }
};

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);

    rclcpp::spin(
        std::make_shared<OmniblueSMC>());

    rclcpp::shutdown();

    return 0;
}