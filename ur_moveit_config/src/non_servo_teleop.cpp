// Copyright 2024 Shin Fang Co., Ltd.
// Author: SFhmichael, Matt Wang
// This file transfers the xbox controller commands to twist and publish to servo node
// We can teleop 2 arm in cartesian space by one xbox controller
#include <control_msgs/msg/joint_jog.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
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
  TeleopCommandPublisher(const std::string& joy_topic)
    : Node("non_servo_teleop")
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
    cmd_pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>(JOINT_TOPIC, 10);
  }
  bool convertJoyToCmd(const std::vector<float>& axes, const std::vector<int>& buttons,
                       std::unique_ptr<std_msgs::msg::Float64MultiArray>& joint)
  {
    bool should_execute = false;
    if ((buttons[LEFT_BUMPER]) || (buttons[RIGHT_BUMPER]))
    {
      double lin_z_up = -(axes[RIGHT_TRIGGER] - AXIS_DEFAULTS.at(RIGHT_TRIGGER));
      double lin_z_down = (axes[LEFT_TRIGGER] - AXIS_DEFAULTS.at(LEFT_TRIGGER));
      joint->data.push_back(lin_z_up + lin_z_down); //elbow
      joint->data.push_back(axes[LEFT_STICK_Y]); //shoulder_lift
      joint->data.push_back(axes[LEFT_STICK_X]); //shoulder_pan
      joint->data.push_back(axes[RIGHT_STICK_Y]); //wrist_1
      joint->data.push_back(axes[RIGHT_STICK_X]); //wrist_2
      double wrist3_positive = buttons[A];
      double wrist3_negative = -1 * (buttons[Y]);
      joint->data.push_back(wrist3_positive + wrist3_negative); //wrist_3


      return true;
    }
    else
    {
      return false;
    }
  }

  void joyCB(const sensor_msgs::msg::Joy::ConstSharedPtr& msg)
  {
    // Create the messages we might publish
    auto joint_msg = std::make_unique<std_msgs::msg::Float64MultiArray>();
    // This call updates the frame for twist commands and start/stop servo
    // Convert the joystick message to Twist or JointJog and publish
    if(convertJoyToCmd(msg->axes, msg->buttons, joint_msg))
    {// publish the JointJog
      cmd_pub_->publish(std::move(joint_msg));
    }
    
  }

private:
  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr cmd_pub_;

  std::string JOINT_TOPIC = "/forward_velocity_controller/commands";
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  std::vector<std::shared_ptr<TeleopCommandPublisher>> nodes;
  nodes.push_back(std::make_shared<TeleopCommandPublisher>("/joy"));

  rclcpp::executors::MultiThreadedExecutor executor;
  for (auto& node : nodes)
  {
    executor.add_node(node);
  }

  executor.spin();

  rclcpp::shutdown();
  return 0;
}