#pragma once

#include "frontier_explorer/frontier_explorer.hpp"

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

#include <optional>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav2_msgs/action/navigate_to_pose.hpp>
#include <std_srvs/srv/trigger.hpp>

class FrontierExplorerNode : public rclcpp::Node
{
public:
    FrontierExplorerNode();

private:
    using NavigateToPose = nav2_msgs::action::NavigateToPose;
    using GoalHandleNav = rclcpp_action::ClientGoalHandle<NavigateToPose>;

    FrontierExplorer explorer_;

    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
    rclcpp_action::Client<NavigateToPose>::SharedPtr nav_client_;
    rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr finish_mapping_client_;
    rclcpp::TimerBase::SharedPtr timer_;

    nav_msgs::msg::OccupancyGrid current_map_;

    bool map_received_;
    bool goal_active_;
    bool finish_requested_;
    std::optional<rclcpp::Time> coverage_threshold_since_;

    std::string map_topic_;
    std::string action_name_;
    std::string finish_mapping_service_;
    double explore_period_;
    double completion_coverage_ratio_;
    double completion_hold_time_;
    double goal_clearance_radius_;
    double goal_search_radius_;
    int no_frontier_count_;
    int no_frontier_finish_count_;
    int min_frontier_cluster_size_;
    bool auto_finish_enabled_;

    void loadParameters();
    void mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg);
    void timerCallback();
    void sendGoal(const geometry_msgs::msg::PoseStamped & goal);
    void requestMappingFinish();
    double calculateCoverageRatio(const nav_msgs::msg::OccupancyGrid & map) const;
    bool shouldFinishByCoverage();
};
