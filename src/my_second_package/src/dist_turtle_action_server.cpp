#include <memory>
#include <thread>
#include <cmath> // 수학 계산용 (sqrt, pow)
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
    // 거북이에게 속도 명령을 내릴 Publisher
    publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("/turtle1/cmd_vel", 10);
    
    // 거북이의 현재 위치를 받아올 Subscriber
    subscriber_ = this->create_subscription<turtlesim::msg::Pose>(
      "/turtle1/pose", 10, std::bind(&DistTurtleServer::pose_callback, this, std::placeholders::_1));

    // 액션 서버 설정
    this->action_server_ = rclcpp_action::create_server<DistTurtle>(
      this, "dist_turtle",
      std::bind(&DistTurtleServer::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
      std::bind(&DistTurtleServer::handle_cancel, this, std::placeholders::_1),
      std::bind(&DistTurtleServer::handle_accepted, this, std::placeholders::_1));

    RCLCPP_INFO(this->get_logger(), "Step 3: Action Server with Logic is ready.");
  }

private:
  // 이전 위치와 현재 위치 사이의 거리를 계산하는 함수
  // 유클리드 거리 공식: $d = \sqrt{(x_2 - x_1)^2 + (y_2 - y_1)^2}$
  double calc_diff_pose() {
    if (is_first_time_) {
      previous_pose_ = current_pose_;
      is_first_time_ = false;
    }
    double diff = std::sqrt(std::pow(current_pose_.x - previous_pose_.x, 2) + 
                            std::pow(current_pose_.y - previous_pose_.y, 2));
    previous_pose_ = current_pose_;
    return diff;
  }

  // 액션의 핵심 로직이 돌아가는 함수
  void execute(const std::shared_ptr<GoalHandleDistTurtle> goal_handle) {
    RCLCPP_INFO(this->get_logger(), "Execution started!");
    const auto goal = goal_handle->get_goal();
    auto feedback = std::make_shared<DistTurtle::Feedback>();
    auto result = std::make_shared<DistTurtle::Result>();
    geometry_msgs::msg::Twist msg;

    // 목표 거리에 도달할 때까지 반복
    while (rclcpp::ok()) {
      total_dist_ += calc_diff_pose();
      feedback->remained_dist = goal->dist - total_dist_;
      goal_handle->publish_feedback(feedback);

      // 로봇 이동 명령
      msg.linear.x = goal->linear_x;
      msg.angular.z = goal->angular_z;
      publisher_->publish(msg);

      // 남은 거리가 0.2미만이면 목표 도착으로 간주
      if (feedback->remained_dist < 0.2) break; 
      
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }

    // 최종 결과 반환 및 변수 초기화
    result->result_dist = total_dist_;
    total_dist_ = 0.0; 
    is_first_time_ = true;
    goal_handle->succeed(result);
    RCLCPP_INFO(this->get_logger(), "Goal reached and succeeded!");
  }

  // 핸들러 함수들
  rclcpp_action::GoalResponse handle_goal(const rclcpp_action::GoalUUID &, std::shared_ptr<const DistTurtle::Goal>) {
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }
  
  rclcpp_action::CancelResponse handle_cancel(const std::shared_ptr<GoalHandleDistTurtle>) {
    return rclcpp_action::CancelResponse::ACCEPT;
  }
  
  void handle_accepted(const std::shared_ptr<GoalHandleDistTurtle> goal_handle) {
    // 멀티스레드를 사용하여 execute 함수 실행 (노드 스핀과 분리)
    std::thread{std::bind(&DistTurtleServer::execute, this, goal_handle)}.detach();
  }
  
  void pose_callback(const turtlesim::msg::Pose::SharedPtr msg) { 
    current_pose_ = *msg; 
  }

  double total_dist_ = 0.0;
  bool is_first_time_ = true;
  turtlesim::msg::Pose current_pose_, previous_pose_;
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