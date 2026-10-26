/**
 * @file Param.cpp
 * @author Oriol Gorriz (origovi2000@gmail.com)
 * @brief Contains the Param class member functions implementation
 * @version 1.0
 * @date 2022-10-31
 *
 * @copyright Copyright (c) 2022 BCN eMotorsport
 */

#include "utils/Params.hpp"

#include <ament_index_cpp/get_package_share_directory.hpp>

/* ----------------------------- Private Methods ---------------------------- */

namespace {
/**
 * @brief Reads a parameter that may already have been declared by
 * automatically_declare_parameters_from_overrides (YAML load), otherwise
 * declares it with the given default.
 */
template <typename T>
T getOrDeclare(rclcpp::Node *node, const std::string &name, const T &def) {
  if (!node->has_parameter(name)) {
    return node->declare_parameter<T>(name, def);
  }
  return node->get_parameter(name).get_value<T>();
}
}  // namespace

/* ----------------------------- Public Methods ----------------------------- */

Params::Params(rclcpp::Node *const node) {
  main.package_path = ament_index_cpp::get_package_share_directory("urinay");
  // Main
  main.input_cones_topic = getOrDeclare(node, "input_cones_topic", std::string("/AS/P/ccat/cones"));
  main.input_pose_topic = getOrDeclare(node, "input_pose_topic", std::string("/AS/C/state"));
  main.output_full_topic = getOrDeclare(node, "output_full_topic", std::string("/AS/P/tracklimits/full"));
  main.output_partial_topic = getOrDeclare(node, "output_partial_topic", std::string("/AS/P/tracklimits/partial"));
  main.shutdown_on_loop_closure = getOrDeclare(node, "shutdown_on_loop_closure", true);
  main.min_cone_confidence = getOrDeclare(node, "min_cone_confidence", 0.0);
  main.verbose = getOrDeclare(node, "verbose", false);

  // WayComputer
  wayComputer.max_triangle_edge_len = getOrDeclare(node, "max_triangle_edge_len", 9.0);
  wayComputer.min_triangle_angle = getOrDeclare(node, "min_triangle_angle", 0.25);
  wayComputer.failsafe_max_way_horizon_size = getOrDeclare(node, "failsafe_max_way_horizon_size", 6);
  wayComputer.general_failsafe = getOrDeclare(node, "general_failsafe", true);
  wayComputer.general_failsafe_safetyFactor = getOrDeclare(node, "general_failsafe_safetyFactor", 1.4);
  // WayComputer::Search
  wayComputer.search.max_way_horizon_size = getOrDeclare(node, "max_way_horizon_size", 0);
  wayComputer.search.tree_search_max_height = getOrDeclare(node, "tree_search_max_height", 7);
  wayComputer.search.search_radius = getOrDeclare(node, "search_radius", 5.0);
  wayComputer.search.max_angle_diff = getOrDeclare(node, "max_angle_diff", 0.6);
  wayComputer.search.min_track_width = getOrDeclare(node, "min_track_width", 2.8);
  wayComputer.search.max_search_options = getOrDeclare(node, "max_search_options", 2);
  wayComputer.search.overnext_midpoint_angle = getOrDeclare(node, "overnext_midpoint_angle", 0.2);
  wayComputer.search.max_next_heuristic = getOrDeclare(node, "max_next_heuristic", 3.0);
  wayComputer.search.heur_dist_weight = getOrDeclare(node, "heur_dist_weight", 0.6);
  wayComputer.search.heur_track_width_diff_weight = getOrDeclare(node, "heur_track_width_diff_weight", 0.2);
  wayComputer.search.allow_intersection = getOrDeclare(node, "allow_intersection", false);
  wayComputer.search.min_dist_between_midpoints = getOrDeclare(node, "min_dist_between_midpoints", 0.09);
  // WayComputer::Trace
  wayComputer.trace.max_dist_loop_closure = getOrDeclare(node, "max_dist_loop_closure", 1.0);
  wayComputer.trace.max_angle_diff_loop_closure = getOrDeclare(node, "max_angle_diff_loop_closure", 0.6);
  wayComputer.trace.vital_num_midpoints = getOrDeclare(node, "vital_num_midpoints", 5);

  // Visualization
  visualization.publish_markers = getOrDeclare(node, "publish_markers", false);
  visualization.visualize_treeSearch = getOrDeclare(node, "visualize_treeSearch", false);
  visualization.triangulation_topic = getOrDeclare(node, "marker_topics.triangulation", std::string("/AS/P/urinay/markers/triangulation"));
  visualization.way_topic = getOrDeclare(node, "marker_topics.way", std::string("/AS/P/urinay/markers/way"));
  visualization.treeSearch_topic = getOrDeclare(node, "marker_topics.treeSearch", std::string("/AS/P/urinay/markers/treeSearch"));
}
