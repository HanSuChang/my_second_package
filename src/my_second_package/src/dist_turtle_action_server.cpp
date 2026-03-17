#include <memory>
#include <thread>
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "turtlesim/msg/pose.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "my_first_package_msgs/action/dist_turtle.hpp"

class DistTurtleServer : public rclcpp::Node {
public:
  using DistTurtle = my_first_package_msgs::action::DistTurtle;
  using GoalHandleDistTurtle = rclcpp_action::ServerGoalHandle<DistTurtle>;

  DistTurtleServer() : Node("dist_turtle_action_server") {
    publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("/turtle1/cmd_vel", 10);
    subscriber_ = this->create_subscription<turtlesim::msg::Pose>(
      "/turtle1/pose", 10, std::bind(&DistTurtleServer::pose_callback, this, std::placeholders::_1));

    // 액션 서버 초기화
    this->action_server_ = rclcpp_action::create_server<DistTurtle>(
      this, "dist_turtle",
      std::bind(&DistTurtleServer::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
      std::bind(&DistTurtleServer::handle_cancel, this, std::placeholders::_1),
      std::bind(&DistTurtleServer::handle_accepted, this, std::placeholders::_1));

    RCLCPP_INFO(this->get_logger(), "Step 2: Action Server Infrastructure is ready.");
  }

private:
  // 목표를 받을지 말지 결정 (여기선 무조건 수락)
  rclcpp_action::GoalResponse handle_goal(const rclcpp_action::GoalUUID &, std::shared_ptr<const DistTurtle::Goal> goal) {
    RCLCPP_INFO(this->get_logger(), "Received goal: dist %f", goal->dist);
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  // 취소 처리
  rclcpp_action::CancelResponse handle_cancel(const std::shared_ptr<GoalHandleDistTurtle>) {
    RCLCPP_INFO(this->get_logger(), "Cancel request received");
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  // 수락 완료 후 실행 스레드 분리
  void handle_accepted(const std::shared_ptr<GoalHandleDistTurtle> goal_handle) {
    std::thread{std::bind(&DistTurtleServer::execute, this, goal_handle)}.detach();
  }

  // 실제 연산이 일어날 곳 (지금은 빈 껍데기)
  void execute(const std::shared_ptr<GoalHandleDistTurtle> goal_handle) {
    RCLCPP_INFO(this->get_logger(), "Execution started...");
    auto result = std::make_shared<DistTurtle::Result>();
    goal_handle->succeed(result);
  }

  void pose_callback(const turtlesim::msg::Pose::SharedPtr msg) { current_pose_ = *msg; }

  turtlesim::msg::Pose current_pose_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
  rclcpp::Subscription<turtlesim::msg::Pose>::SharedPtr subscriber_;
  rclcpp_action::Server<DistTurtle>::SharedPtr action_server_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<DistTurtleServer>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}