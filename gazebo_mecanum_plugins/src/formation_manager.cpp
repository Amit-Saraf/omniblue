#include <iostream>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose2_d.hpp>

class FormationManager : public rclcpp::Node
{
public:

    FormationManager()
    : Node("formation_manager")
    {
        RCLCPP_INFO(get_logger(), "FORMATION MANAGER STARTED");

        goal_pub_ =
            create_publisher<geometry_msgs::msg::Pose2D>(
                "/formation_goal",
                10);
                

        RCLCPP_INFO(get_logger(), "WAITING FOR GOAL INPUT");        
        goal_.x = 10.0;
        goal_.y = 5.0;
        goal_.theta = 0.0;

        RCLCPP_INFO(
            get_logger(),
            "Using fixed goal %.2f %.2f",
            goal_.x,
            goal_.y);

        timer_ =
            create_wall_timer(
                std::chrono::milliseconds(100),
                std::bind(
                    &FormationManager::publishGoal,
                    this));
    }

private:

    geometry_msgs::msg::Pose2D goal_;

    rclcpp::Publisher<
        geometry_msgs::msg::Pose2D>::SharedPtr goal_pub_;

    rclcpp::TimerBase::SharedPtr timer_;

    void publishGoal()
    {
        goal_pub_->publish(goal_);

        RCLCPP_INFO(
            get_logger(),
            "Publishing goal: %.2f %.2f",
            goal_.x,
            goal_.y);
    }
};

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);

    rclcpp::spin(
        std::make_shared<FormationManager>());

    rclcpp::shutdown();

    return 0;
}