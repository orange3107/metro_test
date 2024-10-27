#include "robot.h"
#include "particle_filter.h"
#include <iostream>
#include <unistd.h>
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

using namespace std;


class RobotPositionPublisher : public rclcpp::Node {
public:
    RobotPositionPublisher(const rclcpp::NodeOptions & options)
        : Node("robot_position_publisher") {
        real_position_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("real_position", 10);
        estimated_position_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("estimated_position", 10);
    }

    void publishPositions(double x_real, double y_real, double theta_real,
                          double x_est, double y_est, double theta_est) {
        // Публикуем реальное положение
        auto real_position_msg = geometry_msgs::msg::PoseStamped();
        real_position_msg.header.stamp = this->get_clock()->now();
        real_position_msg.header.frame_id = "map";  // Название фрейма, используемого в rviz2
        real_position_msg.pose.position.x = x_real;
        real_position_msg.pose.position.y = y_real;
        real_position_msg.pose.position.z = 0.0;
        real_position_msg.pose.orientation = createQuaternionMsgFromYaw(theta_real);
        real_position_pub_->publish(real_position_msg);

        // Публикуем предсказанное положение
        auto estimated_position_msg = geometry_msgs::msg::PoseStamped();
        estimated_position_msg.header.stamp = this->get_clock()->now();
        estimated_position_msg.header.frame_id = "map";
        estimated_position_msg.pose.position.x = x_est;
        estimated_position_msg.pose.position.y = y_est;
        estimated_position_msg.pose.position.z = 0.0;
        estimated_position_msg.pose.orientation = createQuaternionMsgFromYaw(theta_est);
        estimated_position_pub_->publish(estimated_position_msg);
    }

private:
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr real_position_pub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr estimated_position_pub_;

    geometry_msgs::msg::Quaternion createQuaternionMsgFromYaw(double yaw) {
        geometry_msgs::msg::Quaternion q;
        q.w = cos(yaw * 0.5);
        q.x = 0.0;
        q.y = 0.0;
        q.z = sin(yaw * 0.5);
        return q;
    }
};

int main(int argc, char **argv) {

    rclcpp::init(argc, argv);
    auto node = std::make_shared<RobotPositionPublisher>(rclcpp::NodeOptions());

    std::vector<std::pair<double, double>> landmarks = {{5, 5}, {10, 10}, {15, 5}, {10, 0}};

    std::vector<std::vector<double>> state = {
        {1, 0.5, 0.2},
        {1, 0.5, 0.2},
        {1, 0.5, 0.2},
        {1, 0.5, 0.2},
        {1, 0.5, 0.2},
        {1, 0.5, 0.2},
        {1, 0.5, 0.2},
        {1, 0.5, 0.2},
        {1, 0.5, 0.2},
        {1, 0.5, 0.2},
    }; // {dt, vel, steer}

    Robot robot(0.0, 0.0, 0.0, 2.5, 0.05, 0.05, 0.1);
    ParticleFilter filter(robot, 1000, 0.1, landmarks);

    for(int i = 0; i < state.size(); ++i){

        robot.update(state[i][0], state[i][1], state[i][2], landmarks);
        filter.update(state[i][0], state[i][1], state[i][2], landmarks, robot);

        double x_est, y_est, th_est;
        filter.getEstimate(x_est, y_est, th_est);

        double x_tr, y_tr, th_tr;
        x_tr = robot.getX();
        y_tr = robot.getY();
        th_tr = robot.getTheta();

        node->publishPositions(x_tr, y_tr, th_tr, x_est, y_est, th_est);

        cout << "Iter: " << i << endl; 
        robot.printState();
        cout << "Estimate: " << "X: " << x_est << " Y: " << y_est << " Theta: " << th_est << endl;

        usleep(state[i][0]*1000000);

        if(!rclcpp::ok()) break;

    }

    rclcpp::shutdown();
    return 0;
}