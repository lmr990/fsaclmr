/**
 * @file Visualization.hpp
 * @author Oriol Gorriz (origovi2000@gmail.com)
 * @brief Contains the Visualization class specification
 * @version 1.0
 * @date 2022-10-31
 *
 * @copyright Copyright (c) 2025 BCN eMotorsport
 */

#pragma once

#include <rclcpp/rclcpp.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <std_msgs/msg/header.hpp>

#include "structures/Trace.hpp"
#include "structures/TraceBuffer.hpp"
#include "utils/Params.hpp"
#include "utils/definitions.hpp"
#include "utils/Logger.hpp"

/**
 * @brief Class that implements all necessary functions to visualize all
 * the program's results.
 */
class Visualization {
 private:
  /**
   * @brief All Markers publishers.
   */
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr trianglesPub, wayPub, traceBufferPub;

  /**
   * @brief All parameters related to the Visualization class.
   */
  Params::Visualization params_;

  /**
   * @brief All Markers will be published with this header.
   */
  std_msgs::msg::Header lastHeader_;

 public:
  /**
   * @brief Construct a new Visualization object.
   */
  Visualization() = default;

  // Singleton pattern
  static Visualization &getInstance();
  Visualization(Visualization const &) = delete;
  void operator=(Visualization const &) = delete;

  /**
   * @brief Method to initialize the Singleton.
   *
   * @param[in] node
   * @param[in] params
   */
  void init(rclcpp::Node *const node, const Params::Visualization &params);

  /**
   * @brief Sets the \a lastHeader_ attribute, all Markers will be published
   * with this header.
   *
   * @param[in] header
   */
  void setHeader(const std_msgs::msg::Header &header);

  /**
   * @brief Method to visualize an EdgeSet.
   *
   * @param[in] edgeSet
   */
  void visualize(const EdgeSet &edgeSet) const;

  /**
   * @brief Method to visualize a Way.
   *
   * @param[in] way
   */
  void visualize(const Trace &way) const;

  /**
   * @brief Method to visualize a TraceBuffer, i.e. all traces in real time.
   * A delay is added to visualize the decision tree process.
   *
   * @param[in] traceBuffer
   */
  void visualize(const TraceBuffer &traceBuffer) const;
};
