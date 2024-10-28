// Copyright 2024 Shin Fang Co., Ltd.
// Author: SFhmichael, Matt Wang
// This file transfers the xbox controller commands to twist and publish to servo node
// We can teleop 2 arm in cartesian space by one xbox controller
#include <control_msgs/msg/joint_jog.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <std_srvs/srv/trigger.hpp>

// xbox controller buttons and axes enum
enum Axis
{
  LEFT_STICK_X = 0,
  LEFT_STICK_Y = 1,
  RIGHT_STICK_X = 2,
  RIGHT_STICK_Y = 3,
  LEFT_TRIGGER = 4,
  RIGHT_TRIGGER = 5,
};
enum Button
{
  A = 0,
  B = 1,
  X = 2,
  Y = 3,
  CHANGE_VIEW = 4,
  MENU = 6,
  LEFT_JOYSTICK = 7,
  RIGHT_JOYSTICK = 8,
  LEFT_BUMPER = 9,
  RIGHT_BUMPER = 10,
  UP = 11,
  DOWN = 12,
  LEFT = 13,
  RIGHT = 14,
};

// Some axes have offsets (e.g. the default trigger position is 0 )
// This will map the default values for the axes
std::map<Axis, double> AXIS_DEFAULTS = { { LEFT_TRIGGER, 0.0 }, { RIGHT_TRIGGER, 0.0 } };
std::map<Button, double> BUTTON_DEFAULTS;

class TeleopCommandPublisher : public rclcpp::Node
{
public:
  TeleopCommandPublisher(const std::string& planning_group_namespace, const std::string& joy_topic)
    : Node("teleop_command_publisher")
  {
    /*
    ns_ = planning_group_namespace;
    TWIST_TOPIC = "/" + ns_ + TWIST_TOPIC;
    JOINT_TOPIC = "/" + ns_ + JOINT_TOPIC;
    EEF_FRAME = ns_ + EEF_FRAME;
    BASE_FRAME = ns_ + BASE_FRAME;
    START_SERVO_SERVICE = ns_ + START_SERVO_SERVICE;
    STOP_SERVO_SERVICE = ns_ + STOP_SERVO_SERVICE;
    */
    joy_sub_ = this->create_subscription<sensor_msgs::msg::Joy>(
        joy_topic, 10, std::bind(&TeleopCommandPublisher::joyCB, this, std::placeholders::_1));
    twist_pub_ = this->create_publisher<geometry_msgs::msg::TwistStamped>(TWIST_TOPIC, 10);
    joint_pub_ = this->create_publisher<control_msgs::msg::JointJog>(JOINT_TOPIC, 10);

    // Create a service client to start the ServoNode
    servo_start_client_ = this->create_client<std_srvs::srv::Trigger>(START_SERVO_SERVICE);
    servo_start_client_->wait_for_service(std::chrono::seconds(1));
    servo_stop_client_ = this->create_client<std_srvs::srv::Trigger>(STOP_SERVO_SERVICE);
    servo_stop_client_->wait_for_service(std::chrono::seconds(1));
    servo_start_client_->async_send_request(std::make_shared<std_srvs::srv::Trigger::Request>());
    servo_start_ = true;
    frame_to_publish_ = BASE_FRAME;
  }
  bool convertJoyToCmd(const std::vector<float>& axes, const std::vector<int>& buttons,
                       std::unique_ptr<geometry_msgs::msg::TwistStamped>& twist,
                       std::unique_ptr<control_msgs::msg::JointJog>& joint)
  {
    bool should_execute = false;
    if ((buttons[LEFT_BUMPER]) || (buttons[RIGHT_BUMPER]))
    {
      should_execute = true;
    }
    if (should_execute)
    {
      twist->twist.linear.x = axes[RIGHT_STICK_Y];
      twist->twist.linear.y = axes[RIGHT_STICK_X];

      double lin_z_up = -(axes[RIGHT_TRIGGER] - AXIS_DEFAULTS.at(RIGHT_TRIGGER));
      double lin_z_down = (axes[LEFT_TRIGGER] - AXIS_DEFAULTS.at(LEFT_TRIGGER));
      twist->twist.linear.z = lin_z_up + lin_z_down;

      twist->twist.angular.y = axes[LEFT_STICK_Y];
      twist->twist.angular.x = axes[LEFT_STICK_X];

      double roll_positive = buttons[UP];
      double roll_negative = -1 * (buttons[DOWN]);
      twist->twist.angular.z = roll_positive + roll_negative;
      return true;
    }
    else
    {
      joint->joint_names.push_back("wrist_3_joint");
      joint->velocities.push_back((buttons[A] - buttons[Y]));
      return false;
    }
  }
  void updateStatusAndCmdFrame(std::string& frame_name, const std::vector<int>& buttons)
  {
    if (buttons[CHANGE_VIEW])
      frame_name = (frame_name == EEF_FRAME) ? BASE_FRAME : EEF_FRAME;
    if (buttons[MENU])
    {
      if (servo_start_)
      {
        servo_stop_client_->async_send_request(std::make_shared<std_srvs::srv::Trigger::Request>());
        servo_start_ = false;
      }
      else
      {
        servo_start_client_->async_send_request(std::make_shared<std_srvs::srv::Trigger::Request>());
        servo_start_ = true;
      }
    }
  }
  void joyCB(const sensor_msgs::msg::Joy::ConstSharedPtr& msg)
  {
    // Create the messages we might publish
    auto twist_msg = std::make_unique<geometry_msgs::msg::TwistStamped>();
    auto joint_msg = std::make_unique<control_msgs::msg::JointJog>();
    // This call updates the frame for twist commands and start/stop servo
    updateStatusAndCmdFrame(frame_to_publish_, msg->buttons);
    // Convert the joystick message to Twist or JointJog and publish
    if (convertJoyToCmd(msg->axes, msg->buttons, twist_msg, joint_msg))
    {
      // publish the TwistStamped
      twist_msg->header.frame_id = frame_to_publish_;
      twist_msg->header.stamp = this->now();
      twist_pub_->publish(std::move(twist_msg));
    }
    else
    {
      // publish the JointJog
      joint_msg->header.stamp = this->now();
      joint_msg->header.frame_id = frame_to_publish_;
      joint_pub_->publish(std::move(joint_msg));
    }
  }

private:
  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr twist_pub_;
  rclcpp::Publisher<control_msgs::msg::JointJog>::SharedPtr joint_pub_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr servo_start_client_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr servo_stop_client_;
  std::string ns_;

  std::string TWIST_TOPIC = "/servo_node/delta_twist_cmds";
  std::string JOINT_TOPIC = "/servo_node/delta_joint_cmds";
  std::string EEF_FRAME = "tool0";
  std::string BASE_FRAME = "base_link";
  std::string START_SERVO_SERVICE = "/servo_node/start_servo";
  std::string STOP_SERVO_SERVICE = "/servo_node/stop_servo";
  std::string frame_to_publish_;
  bool servo_start_;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  std::vector<std::shared_ptr<TeleopCommandPublisher>> nodes;
  nodes.push_back(std::make_shared<TeleopCommandPublisher>("la", "/joy"));
  //nodes.push_back(std::make_shared<TeleopCommandPublisher>("ra", "/joy"));

  rclcpp::executors::MultiThreadedExecutor executor;
  for (auto& node : nodes)
  {
    executor.add_node(node);
  }

  executor.spin();

  rclcpp::shutdown();
  return 0;
}