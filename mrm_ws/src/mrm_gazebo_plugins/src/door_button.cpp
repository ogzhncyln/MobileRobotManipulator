// Copyright 2026 MRM Project. Apache-2.0
//
// DoorButton (Gazebo Classic Model Plugin)
// ----------------------------------------
// Her kapı modeli iki kinematik linkten oluşur:
//   * panel  : kapı kanadı (duvar içine kayarak açılır)
//   * button : kapının sağındaki duvar butonu
//
// Basma algılama ("sanal temas sensörü"):
//   Robotun parmak ucu noktası (trigger_link pozu * trigger_offset) butonun
//   kutusunun (press_margin kadar büyütülmüş) içine girerse buton BASILDI sayılır.
//   gazebo_ros2_control eklem pozisyonlarını doğrudan ayarladığı için (kinematik
//   kol) gerçek temas kuvvetleri güvenilir değildir; geometrik kontrol ise
//   deterministik ve hata ayıklaması kolaydır.
//
// Basıldığında: buton press_depth kadar içeri girer, kapı open_offset yönünde
// open_speed hızıyla kayarak açılır ve açık kalır.
//
// ROS arayüzü:
//   /mrm/doors                (mrm_interfaces/msg/DoorState, 2 Hz)
//   /mrm/<door_name>/open     (std_srvs/srv/Trigger)  - hata ayıklama
//   /mrm/<door_name>/close    (std_srvs/srv/Trigger)  - hata ayıklama / sıfırlama

#include <gazebo/common/Plugin.hh>
#include <gazebo/common/Events.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo_ros/node.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <mrm_interfaces/msg/door_state.hpp>

#include <algorithm>
#include <atomic>
#include <memory>
#include <string>
#include <vector>

namespace mrm_gazebo_plugins
{

class DoorButton : public gazebo::ModelPlugin
{
public:
  void Load(gazebo::physics::ModelPtr model, sdf::ElementPtr sdf) override
  {
    model_ = model;
    world_ = model->GetWorld();
    node_ = gazebo_ros::Node::Get(sdf);

    door_name_ = sdf->Get<std::string>("door_name", model->GetName()).first;
    robot_model_ = sdf->Get<std::string>("robot_model", "mrm_robot").first;
    trigger_link_ = sdf->Get<std::string>("trigger_link", "link5").first;
    trigger_offset_ = sdf->Get<ignition::math::Vector3d>(
      "trigger_offset", ignition::math::Vector3d(0.13, 0, 0)).first;
    open_offset_ = sdf->Get<ignition::math::Vector3d>(
      "open_offset", ignition::math::Vector3d(0, 1.0, 0)).first;
    open_speed_ = sdf->Get<double>("open_speed", 0.25).first;
    button_size_ = sdf->Get<ignition::math::Vector3d>(
      "button_size", ignition::math::Vector3d(0.03, 0.10, 0.10)).first;
    press_margin_ = sdf->Get<double>("press_margin", 0.02).first;
    press_depth_ = sdf->Get<double>("press_depth", 0.012).first;

    panel_ = model->GetLink("panel");
    button_ = model->GetLink("button");
    if (!panel_ || !button_) {
      RCLCPP_ERROR(node_->get_logger(), "[%s] 'panel' veya 'button' linki yok!", door_name_.c_str());
      return;
    }
    panel_closed_ = panel_->WorldPose();
    button_rest_ = button_->WorldPose();

    rclcpp::QoS qos(10);
    qos.reliable();
    state_pub_ = node_->create_publisher<mrm_interfaces::msg::DoorState>("/mrm/doors", qos);

    open_srv_ = node_->create_service<std_srvs::srv::Trigger>(
      "/mrm/" + door_name_ + "/open",
      [this](std::shared_ptr<std_srvs::srv::Trigger::Request>,
      std::shared_ptr<std_srvs::srv::Trigger::Response> res) {
        target_open_ = true;
        res->success = true;
        res->message = door_name_ + " opening";
      });
    close_srv_ = node_->create_service<std_srvs::srv::Trigger>(
      "/mrm/" + door_name_ + "/close",
      [this](std::shared_ptr<std_srvs::srv::Trigger::Request>,
      std::shared_ptr<std_srvs::srv::Trigger::Response> res) {
        target_open_ = false;
        res->success = true;
        res->message = door_name_ + " closing";
      });

    last_time_ = world_->SimTime();
    update_conn_ = gazebo::event::Events::ConnectWorldUpdateBegin(
      std::bind(&DoorButton::OnUpdate, this));
    RCLCPP_INFO(node_->get_logger(), "[%s] DoorButton hazir", door_name_.c_str());
  }

private:
  bool IsPressed()
  {
    auto robot = world_->ModelByName(robot_model_);
    if (!robot) {return false;}
    auto link = robot->GetLink(trigger_link_);
    if (!link) {return false;}
    // Parmak ucu noktası dünya çerçevesinde
    auto tip_world = link->WorldPose().CoordPositionAdd(trigger_offset_);
    // Butonun (dinlenme) çerçevesine dönüştür
    auto tip_local = button_rest_.Rot().RotateVectorReverse(tip_world - button_rest_.Pos());
    auto half = button_size_ * 0.5 + ignition::math::Vector3d(press_margin_, press_margin_, press_margin_);
    return std::abs(tip_local.X()) <= half.X() &&
           std::abs(tip_local.Y()) <= half.Y() &&
           std::abs(tip_local.Z()) <= half.Z();
  }

  void OnUpdate()
  {
    auto now = world_->SimTime();
    double dt = (now - last_time_).Double();
    last_time_ = now;
    if (dt <= 0.0 || dt > 0.5) {dt = 0.001;}

    pressed_ = IsPressed();
    if (pressed_ && !target_open_) {
      RCLCPP_INFO(node_->get_logger(), "[%s] BUTONA BASILDI -> kapi aciliyor", door_name_.c_str());
      target_open_ = true;
    }

    // Kapı açıklık oranını hedefe doğru ilerlet
    double len = open_offset_.Length();
    double step = (len > 1e-6) ? open_speed_ * dt / len : 1.0;
    opening_ = target_open_ ? std::min(1.0, opening_ + step) : std::max(0.0, opening_ - step);

    ignition::math::Pose3d p = panel_closed_;
    p.Pos() += open_offset_ * opening_;
    panel_->SetWorldPose(p);
    panel_->SetLinearVel(ignition::math::Vector3d::Zero);
    panel_->SetAngularVel(ignition::math::Vector3d::Zero);

    // Buton görsel geri bildirimi: basılıyken içeri girer
    ignition::math::Pose3d b = button_rest_;
    if (pressed_) {
      b.Pos() += b.Rot().RotateVector(ignition::math::Vector3d(-press_depth_, 0, 0));
    }
    button_->SetWorldPose(b);
    button_->SetLinearVel(ignition::math::Vector3d::Zero);
    button_->SetAngularVel(ignition::math::Vector3d::Zero);

    // 2 Hz durum yayını
    if ((now - last_pub_).Double() >= 0.5) {
      last_pub_ = now;
      mrm_interfaces::msg::DoorState msg;
      msg.name = door_name_;
      msg.open = opening_ >= 0.999;
      msg.opening = static_cast<float>(opening_);
      msg.button_pressed = pressed_;
      state_pub_->publish(msg);
    }
  }

  gazebo::physics::ModelPtr model_;
  gazebo::physics::WorldPtr world_;
  gazebo::physics::LinkPtr panel_, button_;
  gazebo_ros::Node::SharedPtr node_;
  rclcpp::Publisher<mrm_interfaces::msg::DoorState>::SharedPtr state_pub_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr open_srv_, close_srv_;
  gazebo::event::ConnectionPtr update_conn_;

  std::string door_name_, robot_model_, trigger_link_;
  ignition::math::Vector3d trigger_offset_, open_offset_, button_size_;
  double open_speed_{0.25}, press_margin_{0.02}, press_depth_{0.012};
  ignition::math::Pose3d panel_closed_, button_rest_;
  std::atomic<bool> target_open_{false};
  bool pressed_{false};
  double opening_{0.0};
  gazebo::common::Time last_time_, last_pub_;
};

GZ_REGISTER_MODEL_PLUGIN(DoorButton)

}  // namespace mrm_gazebo_plugins
