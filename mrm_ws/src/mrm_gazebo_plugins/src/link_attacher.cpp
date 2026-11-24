// Copyright 2026 MRM Project. Apache-2.0
//
// LinkAttacher (Gazebo Classic World Plugin)
// ------------------------------------------
// Gazebo'da sürtünmeyle nesne tutmak kararsızdır (nesne kayar, titrer, kontrolsüz hareket eder).
// Bu plugin, gripper fiziksel olarak kapandıktan sonra gripper linki ile nesne
// linki arasına çalışma anında "fixed" bir eklem ekler; bırakırken siler.
//
// Servisler:
//   /mrm/attach  (mrm_interfaces/srv/Attach)
//   /mrm/detach  (mrm_interfaces/srv/Attach)
//
// Güvenlik: eklem oluşturma/silme fizik motoru ile aynı thread'de yapılmalı.
// Bu yüzden servis callback'i isteği kuyruğa koyar, WorldUpdateBegin olayında
// (fizik adımından hemen önce) işlenir ve cevap bir std::promise ile döner.

#include <gazebo/common/Plugin.hh>
#include <gazebo/common/Events.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo_ros/node.hpp>
#include <rclcpp/rclcpp.hpp>
#include <mrm_interfaces/srv/attach.hpp>

#include <deque>
#include <future>
#include <map>
#include <memory>
#include <mutex>
#include <string>

namespace mrm_gazebo_plugins
{

using Attach = mrm_interfaces::srv::Attach;

struct Job
{
  bool attach;
  Attach::Request req;
  std::promise<Attach::Response> promise;
};

class LinkAttacher : public gazebo::WorldPlugin
{
public:
  void Load(gazebo::physics::WorldPtr world, sdf::ElementPtr sdf) override
  {
    world_ = world;
    node_ = gazebo_ros::Node::Get(sdf);

    attach_srv_ = node_->create_service<Attach>(
      "/mrm/attach",
      [this](const std::shared_ptr<Attach::Request> req, std::shared_ptr<Attach::Response> res) {
        *res = Enqueue(true, *req);
      });
    detach_srv_ = node_->create_service<Attach>(
      "/mrm/detach",
      [this](const std::shared_ptr<Attach::Request> req, std::shared_ptr<Attach::Response> res) {
        *res = Enqueue(false, *req);
      });

    update_conn_ = gazebo::event::Events::ConnectWorldUpdateBegin(
      std::bind(&LinkAttacher::OnUpdate, this));
    RCLCPP_INFO(node_->get_logger(), "LinkAttacher hazir: /mrm/attach, /mrm/detach");
  }

private:
  Attach::Response Enqueue(bool attach, const Attach::Request & req)
  {
    auto job = std::make_shared<Job>();
    job->attach = attach;
    job->req = req;
    auto fut = job->promise.get_future();
    {
      std::lock_guard<std::mutex> lk(mutex_);
      jobs_.push_back(job);
    }
    // Simülasyon duraklatılmışsa sonsuza kadar beklememek için zaman aşımı
    if (fut.wait_for(std::chrono::seconds(5)) != std::future_status::ready) {
      Attach::Response r;
      r.success = false;
      r.message = "timeout: simulation paused?";
      return r;
    }
    return fut.get();
  }

  void OnUpdate()
  {
    std::deque<std::shared_ptr<Job>> jobs;
    {
      std::lock_guard<std::mutex> lk(mutex_);
      jobs.swap(jobs_);
    }
    for (auto & job : jobs) {
      job->promise.set_value(job->attach ? DoAttach(job->req) : DoDetach(job->req));
    }
  }

  static std::string Key(const Attach::Request & r)
  {
    return r.model_name_1 + "::" + r.link_name_1 + "|" + r.model_name_2 + "::" + r.link_name_2;
  }

  // model_name_2 boşsa: kontrol noktasına en yakın "piece_" modelini bul.
  // Böylece robot, parçanın simülatördeki adını bilmeden (yalnızca kamerayla
  // bulduğu konuma kıskacı getirerek) tutma yapabilir -> simülatör içi isim bilgisi kullanılmaz.
  std::string FindNearest(const Attach::Request & r)
  {
    auto m1 = world_->ModelByName(r.model_name_1);
    if (!m1) {return "";}
    auto l1 = m1->GetLink(r.link_name_1);
    if (!l1) {return "";}
    ignition::math::Vector3d offset(r.link1_offset.x, r.link1_offset.y, r.link1_offset.z);
    auto p = l1->WorldPose().CoordPositionAdd(offset);
    std::string best;
    double best_d = 1e9;
    for (auto & m : world_->Models()) {
      if (m->GetName().rfind("piece_", 0) != 0) {continue;}
      double d = (m->WorldPose().Pos() - p).Length();
      if (d < best_d) {best_d = d; best = m->GetName();}
    }
    return best;
  }

  Attach::Response DoAttach(const Attach::Request & req_in)
  {
    Attach::Response res;
    Attach::Request r = req_in;
    if (r.model_name_2.empty()) {
      r.model_name_2 = FindNearest(r);
      if (r.model_name_2.empty()) {
        res.message = "no piece found";
        return res;
      }
    }
    auto m1 = world_->ModelByName(r.model_name_1);
    auto m2 = world_->ModelByName(r.model_name_2);
    if (!m1 || !m2) {
      res.message = "model not found";
      return res;
    }
    auto l1 = m1->GetLink(r.link_name_1);
    auto l2 = m2->GetLink(r.link_name_2);
    if (!l1 || !l2) {
      res.message = "link not found";
      return res;
    }
    const std::string key = Key(r);
    if (joints_.count(key)) {
      res.success = true;
      res.message = r.model_name_2;
      return res;
    }

    // Kontrol noktası (örn. parmak ucu orta noktası) ile nesne arasındaki mesafe
    ignition::math::Vector3d offset(r.link1_offset.x, r.link1_offset.y, r.link1_offset.z);
    auto p_check = l1->WorldPose().CoordPositionAdd(offset);
    res.distance = (p_check - l2->WorldPose().Pos()).Length();
    if (r.max_distance > 0.0 && res.distance > r.max_distance) {
      res.success = false;
      res.message = "object too far from gripper: " + std::to_string(res.distance) + " m";
      return res;
    }
    attached_name_ = r.model_name_2;

    // Fixed eklem: ODE, Init() anındaki göreli pozu korur
    auto joint = world_->Physics()->CreateJoint("fixed", m1);
    joint->Attach(l1, l2);
    joint->Load(l1, l2, ignition::math::Pose3d());
    joint->SetModel(m2);
    joint->SetName("mrm_attach_" + r.model_name_2);
    joint->Init();
    joints_[key] = joint;

    // Tutulan nesne parmaklarla çarpışıp titremesin diye çarpışmasını kapat.
    // Bırakıldığında (detach) geri açılır ve kutunun içine düşer.
    l2->SetCollideMode("none");

    res.success = true;
    res.message = r.model_name_2;  // Tutulan modelin adı (detach için)
    RCLCPP_INFO(node_->get_logger(), "ATTACH %s (d=%.3f m)", key.c_str(), res.distance);
    return res;
  }

  Attach::Response DoDetach(const Attach::Request & r)
  {
    Attach::Response res;
    const std::string key = Key(r);
    auto it = joints_.find(key);
    if (it == joints_.end()) {
      res.success = true;  // zaten bağlı değil: idempotent
      res.message = "not attached";
      return res;
    }
    it->second->Detach();
    // Not: Fini() cagrilmaz - fizik adimi icinde kilitlenmeye yol acabiliyor (gozlemlendi);
    // Detach() kisiti kaldirmak icin yeterli (ROS1 gazebo_ros_link_attacher ile ayni)
    joints_.erase(it);

    if (auto m2 = world_->ModelByName(r.model_name_2)) {
      if (auto l2 = m2->GetLink(r.link_name_2)) {
        l2->SetCollideMode("all");
        l2->SetEnabled(true);
      }
    }
    res.success = true;
    res.message = "detached";
    RCLCPP_INFO(node_->get_logger(), "DETACH %s", key.c_str());
    return res;
  }

  gazebo::physics::WorldPtr world_;
  gazebo_ros::Node::SharedPtr node_;
  rclcpp::Service<Attach>::SharedPtr attach_srv_, detach_srv_;
  gazebo::event::ConnectionPtr update_conn_;
  std::mutex mutex_;
  std::deque<std::shared_ptr<Job>> jobs_;
  std::map<std::string, gazebo::physics::JointPtr> joints_;
  std::string attached_name_;
};

GZ_REGISTER_WORLD_PLUGIN(LinkAttacher)

}  // namespace mrm_gazebo_plugins
