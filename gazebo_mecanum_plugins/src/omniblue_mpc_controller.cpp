#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>

#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <Eigen/Dense>
#include <sensor_msgs/msg/laser_scan.hpp>

using std::placeholders::_1;

class OmniblueMPC : public rclcpp::Node
{
public:
    OmniblueMPC() : Node("omniblue_mpc_controller")
    {
        odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
            "/omniblue/odom", 10,
            std::bind(&OmniblueMPC::odomCallback, this, _1));

        scan_sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
            "/omniblue/scan",
            10,
            std::bind(&OmniblueMPC::scanCallback, this, _1));        

        cmd_pub_ = create_publisher<geometry_msgs::msg::Twist>(
            "/omniblue/cmd_vel", 10);

        timer_ = create_wall_timer(
            std::chrono::milliseconds(50),
            std::bind(&OmniblueMPC::controlLoop, this));

        std::cout << "\nEnter Goal X: ";
        std::cin >> goal_x_;

        std::cout << "Enter Goal Y: ";
        std::cin >> goal_y_;

        std::cout << "Enter Goal Theta(rad): ";
        std::cin >> goal_theta_;
    }

private:

    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;

    double x_ = 0.0;
    double y_ = 0.0;
    double theta_ = 0.0;
    double goal_x_ = 0.0;
    double goal_y_ = 0.0;
    double goal_theta_ = 0.0;
    double obstacle_distance_ = 100.0;
    double obstacle_angle_ = 0.0;
    int avoidance_side_ = 0;
    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        x_ = msg->pose.pose.position.x;
        y_ = msg->pose.pose.position.y;

        tf2::Quaternion q(
            msg->pose.pose.orientation.x,
            msg->pose.pose.orientation.y,
            msg->pose.pose.orientation.z,
            msg->pose.pose.orientation.w);

        tf2::Matrix3x3 m(q);

        double roll, pitch;
        m.getRPY(roll, pitch, theta_);
    }

    void scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
    {
        obstacle_distance_ = 100.0;
        obstacle_angle_ = 0.0;

        for(size_t i = 0; i < msg->ranges.size(); i++)
        {
            double r = msg->ranges[i];

            if(std::isnan(r) || std::isinf(r))
                continue;

            double angle =
                msg->angle_min + i * msg->angle_increment;


            if(r < obstacle_distance_)
            {
                obstacle_distance_ = r;
                obstacle_angle_ = angle;
            }
        }
    }

    double saturate(double val, double max_val)
    {
        if(val > max_val) return max_val;
        if(val < -max_val) return -max_val;
        return val;
    }

    void controlLoop()
    {
        geometry_msgs::msg::Twist cmd;

        double dt = 0.1;
        int N = 20;

        Eigen::Vector3d x;
        x << x_, y_, theta_;

        Eigen::Vector3d x_ref;
        x_ref << goal_x_, goal_y_, goal_theta_;

        Eigen::Matrix3d A;
        A.setIdentity();

        Eigen::Matrix<double,3,3> B;

        B << dt*cos(theta_), -dt*sin(theta_), 0,
            dt*sin(theta_),  dt*cos(theta_), 0,
            0,               0,              dt;

        Eigen::Matrix3d Q;
        Q << 20,0,0,
            0,20,0,
            0,0,10;

        Eigen::Matrix3d R;
        R << 0.1,0,0,
            0,0.1,0,
            0,0,0.1;

        Eigen::Vector3d e = x_ref - x;

        Eigen::Matrix2d Rot;

        Rot << cos(theta_),  sin(theta_),
            -sin(theta_),  cos(theta_);

        Eigen::Vector2d err_world;
        err_world << e(0), e(1);

        Eigen::Vector2d err_body;

        err_body = Rot * err_world;

        e(0) = err_body(0);
        e(1) = err_body(1);

        int nx = 3;
        int nu = 3;

        Eigen::MatrixXd F(nx * N, nx);
        Eigen::MatrixXd G(nx * N, nu * N);

        F.setZero();
        G.setZero();

        Eigen::Matrix3d A_power;

        A_power.setIdentity();

        for(int i = 0; i < N; i++)
        {
            A_power = A_power * A;

            F.block(i * nx, 0, nx, nx) = A_power;
        }

        for(int row = 0; row < N; row++)
        {
            for(int col = 0; col <= row; col++)
            {
                Eigen::Matrix3d A_tmp;

                A_tmp.setIdentity();

                for(int k = 0; k < (row - col); k++)
                {
                    A_tmp = A_tmp * A;
                }

                G.block(row * nx, col * nu, nx, nu)
                    = A_tmp * B;
            }
        }

        Eigen::VectorXd X_ref(nx * N);

        for(int i = 0; i < N; i++)
        {
            X_ref.segment(i * nx, nx) = x_ref;
        }

        Eigen::MatrixXd Q_bar(nx * N, nx * N);
        Eigen::MatrixXd R_bar(nu * N, nu * N);

        Q_bar.setZero();
        R_bar.setZero();

        for(int i = 0; i < N; i++)
        {
            Q_bar.block(i * nx, i * nx, nx, nx) = Q;

            R_bar.block(i * nu, i * nu, nu, nu) = R;
        }

        Eigen::MatrixXd H;
        Eigen::VectorXd f;

        H = G.transpose() * Q_bar * G + R_bar;

        f = G.transpose() * Q_bar * (F * x - X_ref);

        double safe_distance = 1.5;

        if(avoidance_side_ == 0)
        {
            if(obstacle_angle_ > 0.0)
            {
                avoidance_side_ = -1;
            }
            else
            {
                avoidance_side_ = 1;
            }
        }

        for(int i = 0; i < N; i++)
        {
            double pred_x =
                (F * x)(i * nx + 0);

            double pred_y =
                (F * x)(i * nx + 1);

            double obs_x =
                x_ + obstacle_distance_ *
                cos(theta_ + obstacle_angle_);

            double obs_y =
                y_ + obstacle_distance_ *
                sin(theta_ + obstacle_angle_);

            double dx = pred_x - obs_x;
            double dy = pred_y - obs_y;

            double dist =
                sqrt(dx*dx + dy*dy);

            if(dist < safe_distance)
            {
                double penalty =
                    8890.0 *
                    (safe_distance - dist);

                double tangent_x =
                    -sin(obstacle_angle_);

                double tangent_y =
                    cos(obstacle_angle_);

                f(i * nu + 0) +=
                    avoidance_side_ *
                    penalty *
                    tangent_x;

                f(i * nu + 1) +=
                    avoidance_side_ *
                    penalty *
                    tangent_y;
            }
        }

        Eigen::VectorXd U(nu * N);
        U.setZero();

        double alpha = 0.01;

        for(int iter = 0; iter < 50; iter++)
        {
            Eigen::VectorXd grad;

            grad = H * U + f;

            U = U - alpha * grad;

            for(int i = 0; i < nu * N; i++)
            {
                if(i % 3 == 2)
                {
                    U(i) = saturate(U(i), 1.0);
                }
                else
                {
                    U(i) = saturate(U(i), 0.8);
                }
            }
        }



        double vx = saturate(U(0), 0.8);
        double vy = saturate(U(1), 0.8);
        double wz = saturate(U(2), 1.0);

        cmd.linear.x = vx;
        cmd.linear.y = vy;
        cmd.angular.z = wz;

        cmd_pub_->publish(cmd);
    }
};

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<OmniblueMPC>());
    rclcpp::shutdown();
    return 0;
}
