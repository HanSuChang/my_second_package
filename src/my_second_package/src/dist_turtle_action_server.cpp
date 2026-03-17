#include <memory>
#include <thread>
#include <cmath>
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "turtlesim/msg/pose.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "turtlesim/srv/teleport_absolute.hpp"
#include "my_first_package_msgs/action/dist_turtle.hpp"

class DistTurtleServer : public rclcpp::Node {
public:
  using DistTurtle = my_first_package_msgs::action::DistTurtle;
  using GoalHandleDistTurtle = rclcpp_action::ServerGoalHandle<DistTurtle>;

  DistTurtleServer() : Node("dist_turtle_action_server") {
    publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("/turtle1/cmd_vel", 10);
    subscriber_ = this->create_subscription<turtlesim::msg::Pose>(
      "/turtle1/pose", 10, std::bind(&DistTurtleServer::pose_callback, this, std::placeholders::_1));
    teleport_client_ = this->create_client<turtlesim::srv::TeleportAbsolute>("/turtle1/teleport_absolute");
    
    this->action_server_ = rclcpp_action::create_server<DistTurtle>(
      this, "dist_turtle",
      std::bind(&DistTurtleServer::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
      std::bind(&DistTurtleServer::handle_cancel, this, std::placeholders::_1),
      std::bind(&DistTurtleServer::handle_accepted, this, std::placeholders::_1));
    RCLCPP_INFO(this->get_logger(), "Wall-E Turtle Server is ready!");
  }

private:
  // 1. 특정 방향(Theta)으로 정밀 회전
  void rotate_to_theta(double target_theta) {
    geometry_msgs::msg::Twist msg;
    const double Kp = 4.0;
    while (rclcpp::ok()) {
      double error = target_theta - current_pose_.theta;
      // 각도 차이 최적화 (-PI ~ PI)
      while (error > M_PI) error -= 2 * M_PI;
      while (error < -M_PI) error += 2 * M_PI;

      if (std::abs(error) < 0.005) break;

      msg.angular.z = Kp * error;
      publisher_->publish(msg);
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    stop_turtle();
    std::this_thread::sleep_for(std::chrono::milliseconds(200));
  }

  // 2. 조건 만족할 때까지 전진 (벽 감지 또는 좌표 도달)
  void move_until(std::function<bool()> condition, double linear_v) {
    geometry_msgs::msg::Twist msg;
    msg.linear.x = linear_v;
    while (rclcpp::ok() && !condition()) {
      publisher_->publish(msg);
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    stop_turtle();
  }

  void stop_turtle() { geometry_msgs::msg::Twist msg; publisher_->publish(msg); }
  void pose_callback(const turtlesim::msg::Pose::SharedPtr msg) { current_pose_ = *msg; }

  void execute(const std::shared_ptr<GoalHandleDistTurtle> goal_handle) {
    // [0] 시작 위치 강제 초기화 및 저장
    auto request = std::make_shared<turtlesim::srv::TeleportAbsolute::Request>();
    request->x = 5.5; request->y = 5.5; request->theta = 0.0;
    teleport_client_->async_send_request(request);
    std::this_thread::sleep_for(std::chrono::seconds(1));

    double start_x = current_pose_.x;
    double start_y = current_pose_.y;
    RCLCPP_INFO(this->get_logger(), "Home set to: (%.2f, %.2f)", start_x, start_y);

    // [1] 오른쪽 벽까지
    RCLCPP_INFO(this->get_logger(), "Step 1: To the Right Wall");
    rotate_to_theta(0.0);
    move_until([this]() { return current_pose_.x > 10.5; }, 2.0);

    // [2] 천장까지
    RCLCPP_INFO(this->get_logger(), "Step 2: To the Ceiling");
    rotate_to_theta(M_PI / 2.0);
    move_until([this]() { return current_pose_.y > 10.5; }, 2.0);

    // [3] 왼쪽 벽까지
    RCLCPP_INFO(this->get_logger(), "Step 3: To the Left Wall");
    rotate_to_theta(M_PI);
    move_until([this]() { return current_pose_.x < 0.5; }, 2.0);

    // [4] 바닥까지
    RCLCPP_INFO(this->get_logger(), "Step 4: To the Floor");
    rotate_to_theta(-M_PI / 2.0);
    move_until([this]() { return current_pose_.y < 0.5; }, 2.0);

    // [5] 시작 X좌표까지 (오른쪽으로 꺾기)
    RCLCPP_INFO(this->get_logger(), "Step 5: Returning to Start X Line");
    rotate_to_theta(0.0);
    move_until([this, start_x]() { return current_pose_.x >= start_x; }, 2.0);

    // [6] 시작 Y좌표까지 (위로 꺾기)
    RCLCPP_INFO(this->get_logger(), "Step 6: Returning Home (Start Y)");
    rotate_to_theta(M_PI / 2.0);
    move_until([this, start_y]() { return current_pose_.y >= start_y; }, 1.0);

    RCLCPP_INFO(this->get_logger(), "I'm Back Home! Mission Success.");
    goal_handle->succeed(std::make_shared<DistTurtle::Result>());
  }

  // 핸들러 및 멤버 변수
  rclcpp_action::GoalResponse handle_goal(const rclcpp_action::GoalUUID &, std::shared_ptr<const DistTurtle::Goal>) { return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE; }
  rclcpp_action::CancelResponse handle_cancel(const std::shared_ptr<GoalHandleDistTurtle>) { return rclcpp_action::CancelResponse::ACCEPT; }
  void handle_accepted(const std::shared_ptr<GoalHandleDistTurtle> goal_handle) { std::thread{std::bind(&DistTurtleServer::execute, this, goal_handle)}.detach(); }

  turtlesim::msg::Pose current_pose_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
  rclcpp::Subscription<turtlesim::msg::Pose>::SharedPtr subscriber_;
  rclcpp_action::Server<DistTurtle>::SharedPtr action_server_;
  rclcpp::Client<turtlesim::srv::TeleportAbsolute>::SharedPtr teleport_client_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<DistTurtleServer>());
  rclcpp::shutdown();
  return 0;
}