// corobo_arm_drive.cpp -- velocity-controlled (motor-like) 2-DOF arm drive.
// Subscribes to two Twist topics; angular.z of each is the target joint
// velocity (rad/s). Uses ODE's joint motor (fmax + vel params), so it
// behaves like a real motor with a torque limit -- NOT a position servo.
#include <gazebo/common/Plugin.hh>
#include <gazebo/physics/Model.hh>
#include <gazebo/physics/Joint.hh>
#include <gazebo_ros/node.hpp>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <mutex>

namespace gazebo
{
class ArmTwistDrive : public ModelPlugin
{
public:
  void Load(physics::ModelPtr model, sdf::ElementPtr sdf) override
  {
    model_ = model;

    std::string base_joint_name  = sdf->Get<std::string>("base_joint", "base_joint").first;
    std::string elbow_joint_name = sdf->Get<std::string>("elbow_joint", "elbow_joint").first;
    max_effort_base_  = sdf->Get<double>("base_max_effort", 5.0).first;
    max_effort_elbow_ = sdf->Get<double>("elbow_max_effort", 3.0).first;

    base_joint_  = model_->GetJoint(base_joint_name);
    elbow_joint_ = model_->GetJoint(elbow_joint_name);
    if (!base_joint_ || !elbow_joint_) {
      gzerr << "[corobo_arm_drive] joint not found, plugin disabled\n";
      return;
    }

    ros_node_ = gazebo_ros::Node::Get(sdf);

    base_sub_ = ros_node_->create_subscription<geometry_msgs::msg::Twist>(
      "cmd_vel_basejoint", 10,
      [this](geometry_msgs::msg::Twist::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(mutex_);
        base_vel_cmd_ = msg->angular.z;
      });

    elbow_sub_ = ros_node_->create_subscription<geometry_msgs::msg::Twist>(
      "cmd_vel_elbowjoint", 10,
      [this](geometry_msgs::msg::Twist::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(mutex_);
        elbow_vel_cmd_ = msg->angular.z;
      });

    update_connection_ = event::Events::ConnectWorldUpdateBegin(
      std::bind(&ArmTwistDrive::OnUpdate, this));

    RCLCPP_INFO(ros_node_->get_logger(),
      "corobo_arm_drive loaded: base_joint=%s elbow_joint=%s",
      base_joint_name.c_str(), elbow_joint_name.c_str());
  }

  void OnUpdate()
  {
    double bv, ev;
    { std::lock_guard<std::mutex> lock(mutex_); bv = base_vel_cmd_; ev = elbow_vel_cmd_; }
    base_joint_->SetParam("fmax", 0, max_effort_base_);
    base_joint_->SetParam("vel", 0, bv);
    elbow_joint_->SetParam("fmax", 0, max_effort_elbow_);
    elbow_joint_->SetParam("vel", 0, ev);
  }

private:
  physics::ModelPtr model_;
  physics::JointPtr base_joint_, elbow_joint_;
  double max_effort_base_{5.0}, max_effort_elbow_{3.0};
  double base_vel_cmd_{0.0}, elbow_vel_cmd_{0.0};
  std::mutex mutex_;
  gazebo_ros::Node::SharedPtr ros_node_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr base_sub_, elbow_sub_;
  event::ConnectionPtr update_connection_;
};
GZ_REGISTER_MODEL_PLUGIN(ArmTwistDrive)
}  // namespace gazebo
