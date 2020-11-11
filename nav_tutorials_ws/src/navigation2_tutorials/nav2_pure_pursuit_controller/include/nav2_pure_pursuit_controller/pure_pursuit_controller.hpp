/*
 * SPDX-License-Identifier: BSD-3-Clause
 *
 *  Author(s): Shrijit Singh <shrijitsingh99@gmail.com>
 *
 */

#ifndef NAV2_PURE_PURSUIT_CONTROLLER_HPP_
#define NAV2_PURE_PURSUIT_CONTROLLER_HPP_

#include <string>
#include <vector>

#include "nav2_core/controller.hpp"
#include "rclcpp/rclcpp.hpp"
#include "pluginlib/class_loader.hpp"
#include "pluginlib/class_list_macros.hpp"

namespace nav2_pure_pursuit_controller {

  class PurePursuitController: public nav2_core::Controller
  {
public:
    PurePursuitController() = default;
    ~PurePursuitController() override = default;

    void configure(
      const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
      std::string name, const std::shared_ptr < tf2_ros::Buffer > & tf,
      const std::shared_ptr < nav2_costmap_2d::Costmap2DROS > & costmap_ros);


    void cleanup() override;
    void activate() override;
    void deactivate() override;

    geometry_msgs::msg::TwistStamped computeVelocityCommands(
      const geometry_msgs::msg::PoseStamped & pose,
      const geometry_msgs::msg::Twist & velocity) override;

    void setPlan(const nav_msgs::msg::Path & path) override;

protected:
    nav_msgs::msg::Path transformGlobalPlan(const nav_msgs::msg::Path & path, const char* frame_id_to);

    bool transformPose(
      const std::shared_ptr < tf2_ros::Buffer > tf,
      const std::string frame,
      const geometry_msgs::msg::PoseStamped & in_pose,
      geometry_msgs::msg::PoseStamped & out_pose,
      rclcpp::Duration & transform_tolerance
    );

    rclcpp_lifecycle::LifecycleNode::WeakPtr node_;
    std::shared_ptr < tf2_ros::Buffer > tf_;
    std::string plugin_name_;
    std::shared_ptr < nav2_costmap_2d::Costmap2DROS > costmap_ros_;
    rclcpp::Logger logger_ {rclcpp::get_logger("PurePursuitController")};
    rclcpp::Clock::SharedPtr clock_;

    double desired_linear_vel_;
    double lookahead_dist_;
    double max_angular_vel_;
    rclcpp::Duration transform_tolerance_ {0, 0};

    nav_msgs::msg::Path global_plan_;
    nav_msgs::msg::Path local_plan_;
  };

}

#endif //NAV2_PURE_PURSUIT_CONTROLLER_HPP_
