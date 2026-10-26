/**
 * @file main.cpp
 * @author Oriol Gorriz (origovi2000@gmail.com)
 * @brief Main file of Urinay, creates all modules, subcribers and publishers.
 * @version 1.0
 * @date 2022-10-31
 *
 * @copyright Copyright (c) 2022 BCN eMotorsport
 */

#include <driverless_msgs/msg/cone_with_id.hpp>
#include <driverless_msgs/msg/cone_with_id_array.hpp>
#include <driverless_msgs/msg/path_limits.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <sys/stat.h>

#include <iostream>
#include <memory>

#include "modules/DelaunayTri.hpp"
#include "modules/Visualization.hpp"
#include "modules/WayComputer.hpp"
#include "utils/Logger.hpp"

class UrinayNode : public rclcpp::Node {
 public:
  UrinayNode() : Node("urinay", rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true)) {
    params = new Params(this);
    wayComputer = new WayComputer(params->wayComputer);
    Logger::print_immediately = false;
    Visualization::getInstance().init(this, params->visualization);

    use_tf_pose_ = this->has_parameter("use_tf_pose")
                       ? this->get_parameter("use_tf_pose").as_bool()
                       : this->declare_parameter<bool>("use_tf_pose", true);
    mapFrame_ = this->has_parameter("map_frame")
                    ? this->get_parameter("map_frame").as_string()
                    : this->declare_parameter<std::string>("map_frame", "track");
    baseFrame_ = this->has_parameter("base_frame")
                     ? this->get_parameter("base_frame").as_string()
                     : this->declare_parameter<std::string>("base_frame", "base_link");

    rclcpp::QoS partialQos(1);
    // The full path is latched like in the ROS1 version so late subscribers
    // (e.g. the path bridge) always get the last closed-loop result.
    rclcpp::QoS fullQos(1);
    fullQos.transient_local();

    pubPartial = this->create_publisher<driverless_msgs::msg::PathLimits>(params->main.output_partial_topic, partialQos);
    pubFull = this->create_publisher<driverless_msgs::msg::PathLimits>(params->main.output_full_topic, fullQos);

    // The cone-map adapters in this stack publish best-effort (QOS_LATEST).
    rclcpp::QoS conesQos = rclcpp::QoS(rclcpp::KeepLast(1)).best_effort();
    subCones = this->create_subscription<driverless_msgs::msg::ConeWithIdArray>(
        params->main.input_cones_topic, conesQos,
        std::bind(&UrinayNode::mapCallback, this, std::placeholders::_1));

    if (use_tf_pose_) {
      tfBuffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
      tfListener_ = std::make_shared<tf2_ros::TransformListener>(*tfBuffer_);
      // Feed the vehicle pose to WayComputer at 50 Hz from the TF tree.
      tfTimer_ = this->create_wall_timer(
          std::chrono::milliseconds(20),
          std::bind(&UrinayNode::tfPoseCallback, this));
    } else {
      subPose = this->create_subscription<nav_msgs::msg::Odometry>(
          params->main.input_pose_topic, 1,
          std::bind(&WayComputer::stateCallback, wayComputer, std::placeholders::_1));
    }
  }

  ~UrinayNode() {
    delete wayComputer;
    delete params;
  }

 private:
  WayComputer *wayComputer;
  Params *params;

  rclcpp::Publisher<driverless_msgs::msg::PathLimits>::SharedPtr pubPartial;
  rclcpp::Publisher<driverless_msgs::msg::PathLimits>::SharedPtr pubFull;
  rclcpp::Subscription<driverless_msgs::msg::ConeWithIdArray>::SharedPtr subCones;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr subPose;

  bool use_tf_pose_;
  std::string mapFrame_, baseFrame_;
  std::unique_ptr<tf2_ros::Buffer> tfBuffer_;
  std::shared_ptr<tf2_ros::TransformListener> tfListener_;
  rclcpp::TimerBase::SharedPtr tfTimer_;

  void tfPoseCallback() {
    geometry_msgs::msg::TransformStamped tf;
    try {
      tf = tfBuffer_->lookupTransform(mapFrame_, baseFrame_, tf2::TimePointZero);
    } catch (const tf2::TransformException &e) {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 3000,
                           "TF %s->%s failed: %s", mapFrame_.c_str(), baseFrame_.c_str(), e.what());
      return;  // keep last known pose; mapCallback warns if never received
    }
    auto odom = std::make_shared<nav_msgs::msg::Odometry>();
    odom->pose.pose.position.x = tf.transform.translation.x;
    odom->pose.pose.position.y = tf.transform.translation.y;
    odom->pose.pose.position.z = tf.transform.translation.z;
    odom->pose.pose.orientation = tf.transform.rotation;
    wayComputer->stateCallback(odom);
  }

  // This is the map callback
  void mapCallback(const driverless_msgs::msg::ConeWithIdArray::SharedPtr data) {
    if (not wayComputer->isLocalTfValid()) {
      Logger::logwarn("CarState not being received.");
      if (params->main.verbose)
        Logger::print_report();
      return;
    }
    if (data->cones.empty()) {
      Logger::logwarn("Reading empty set of cones.");
      if (params->main.verbose)
        Logger::print_report();
      return;
    }

    Logger::tick("total");

    // Convert to Node vector (::Node is urinay's Node, not rclcpp::Node)
    std::vector<::Node> nodes;
    nodes.reserve(data->cones.size());
    for (const driverless_msgs::msg::ConeWithId &c : data->cones) {
      nodes.emplace_back(c);
    }

    // Update local coordinates of Nodes (makes original local coords unnecessary)
    for (const ::Node &n : nodes) {
      n.updateLocal(wayComputer->getLocalTf());
    }

    // Delaunay triangulation
    TriangleSet triangles = DelaunayTri::compute(nodes);

    // Update the way with the new triangulation
    wayComputer->update(triangles, data->header);

    // Publish loop and write tracklimits to a file
    if (wayComputer->isLoopClosed()) {
      pubFull->publish(wayComputer->getPathLimits());
      Logger::loginfo("Loop closed!");
      std::string loopDir = params->main.package_path + "/loops";
      mkdir(loopDir.c_str(), 0777);
      wayComputer->writeWayToFile(loopDir + "/loop.unay");
      if (params->main.shutdown_on_loop_closure) {
        Logger::tock("total");
        Logger::loginfo("Have a good day :)");
        if (params->main.verbose)
          Logger::print_report();
        rclcpp::shutdown();
      }
    }
    // Publish partial
    else {
      pubPartial->publish(wayComputer->getPathLimits());
    }

    Logger::tock("total");
    if (params->main.verbose)
      Logger::print_report();
  }
};

// Main
int main(int argc, char **argv) {
  rclcpp::init(argc, argv);

  rclcpp::spin(std::make_shared<UrinayNode>());
  rclcpp::shutdown();
}
