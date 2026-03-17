#include <memory>
#include <thread>
#include <cmath>
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "turtlesim/msg/pose.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "my_first_package_msgs/action/dist_turtle.hpp"
#include "rcl_interfaces/msg/set_parameters_result.hpp" // 파라미터 결과용 헤더

class DistTurtleServer : public rclcpp::Node {
public:
  using DistTurtle = my_first_package_msgs::action::DistTurtle;
  using GoalHandleDistTurtle = rclcpp_action::ServerGoalHandle<DistTurtle>;

  DistTurtleServer() : Node("dist_turtle_action_server") {
    publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("/turtle1/cmd_vel", 10);
    subscriber_ = this->create_subscription<turtlesim::msg::Pose>(
      "/turtle1/pose", 10, std::bind(&DistTurtleServer::pose_callback, this, std::placeholders::_1));

    // 1. 파라미터 선언 (기본값 설정)
    this->declare_parameter("quantile_time", 0.75);
    this->get_parameter("quantile_time", quantile_time_);

    // 파라미터 변경 시 호출될 콜백 등록
    parameter_callback_handle_ = this->add_on_set_parameters_callback(
      std::bind(&DistTurtleServer::parameter_callback, this, std::placeholders::_1));

    this->action_server_ = rclcpp_action::create_server<DistTurtle>(
      this, "dist_turtle",
      std::bind(&DistTurtleServer::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
      std::bind(&DistTurtleServer::handle_cancel, this, std::placeholders::_1),
      std::bind(&DistTurtleServer::handle_accepted, this, std::placeholders::_1));

    RCLCPP_INFO(this->get_logger(), "Step 4: Action Server with Parameters is ready.");
  }

private:
  // 파라미터 변경 처리 함수
  rcl_interfaces::msg::SetParametersResult parameter_callback(const std::vector<rclcpp::Parameter> &params) {
    auto result = rcl_interfaces::msg::SetParametersResult();
    result.successful = true;
    for (const auto &param : params) {
      if (param.get_name() == "quantile_time") {
        quantile_time_ = param.as_double();
        RCLCPP_INFO(this->get_logger(), "Parameter 'quantile_time' changed to: %.2f", quantile_time_);
      }
    }
    return result;
  }

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

  void execute(const std::shared_ptr<GoalHandleDistTurtle> goal_handle) {
    const auto goal = goal_handle->get_goal();
    auto feedback = std::make_shared<DistTurtle::Feedback>();
    auto result = std::make_shared<DistTurtle::Result>();
    geometry_msgs::msg::Twist msg;

    while (rclcpp::ok()) {
      total_dist_ += calc_diff_pose();
      feedback->remained_dist = goal->dist - total_dist_;
      goal_handle->publish_feedback(feedback);

      // 중간 지점 알림 로직 (파이썬의 tmp < 0.02 부분)
      double target_point = goal->dist * quantile_time_;
      if (std::abs(total_dist_ - target_point) < 0.05) {
        RCLCPP_INFO(this->get_logger(), "The turtle passes the %.2f point!", quantile_time_);
      }

      msg.linear.x = goal->linear_x;
      msg.angular.z = goal->angular_z;
      publisher_->publish(msg);

      if (feedback->remained_dist < 0.2) break;
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }

    // 2. 결과 데이터 상세화 (좌표 추가)
    result->pos_x = current_pose_.x;
    result->pos_y = current_pose_.y;
    result->pos_theta = current_pose_.theta;
    result->result_dist = total_dist_;

    total_dist_ = 0.0;
    is_first_time_ = true;
    goal_handle->succeed(result);
    RCLCPP_INFO(this->get_logger(), "Goal Reached! Final Pos: (%.2f, %.2f)", result->pos_x, result->pos_y);
  }

  // 핸들러 및 기타 함수들 (동일)
  rclcpp_action::GoalResponse handle_goal(const rclcpp_action::GoalUUID &, std::shared_ptr<const DistTurtle::Goal>) { return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE; }
  rclcpp_action::CancelResponse handle_cancel(const std::shared_ptr<GoalHandleDistTurtle>) { return rclcpp_action::CancelResponse::ACCEPT; }
  void handle_accepted(const std::shared_ptr<GoalHandleDistTurtle> goal_handle) { std::thread{std::bind(&DistTurtleServer::execute, this, goal_handle)}.detach(); }
  void pose_callback(const turtlesim::msg::Pose::SharedPtr msg) { current_pose_ = *msg; }

  double total_dist_ = 0.0;
  double quantile_time_ = 0.75; // 파라미터 변수
  bool is_first_time_ = true;
  turtlesim::msg::Pose current_pose_, previous_pose_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
  rclcpp::Subscription<turtlesim::msg::Pose>::SharedPtr subscriber_;
  rclcpp_action::Server<DistTurtle>::SharedPtr action_server_;
  OnSetParametersCallbackHandle::SharedPtr parameter_callback_handle_; // 파라미터 핸들러
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<DistTurtleServer>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}