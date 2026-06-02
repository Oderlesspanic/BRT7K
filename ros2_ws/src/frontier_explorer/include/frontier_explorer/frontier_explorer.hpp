#pragma once

#include <nav_msgs/msg/occupancy_grid.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <vector>
#include <optional>

struct FrontierPoint
{
    int x;
    int y;
};

class FrontierExplorer
{
public:
    explicit FrontierExplorer();

    std::optional<FrontierPoint> findFrontier(
        const nav_msgs::msg::OccupancyGrid & map,
        int goal_clearance_cells,
        int goal_search_radius_cells,
        int min_frontier_cluster_size
    );

    geometry_msgs::msg::PoseStamped mapToPose(
        const nav_msgs::msg::OccupancyGrid & map,
        const FrontierPoint & point
    );

private:
    bool isFree(
        const nav_msgs::msg::OccupancyGrid & map,
        int x,
        int y
    );

    bool isUnknown(
        const nav_msgs::msg::OccupancyGrid & map,
        int x,
        int y
    );

    bool isInBounds(
        const nav_msgs::msg::OccupancyGrid & map,
        int x,
        int y
    );

    bool isFrontier(
        const nav_msgs::msg::OccupancyGrid & map,
        int x,
        int y
    );

    bool hasClearance(
        const nav_msgs::msg::OccupancyGrid & map,
        int x,
        int y,
        int radius_cells
    );

    std::optional<FrontierPoint> findSafeGoalNear(
        const nav_msgs::msg::OccupancyGrid & map,
        const std::vector<FrontierPoint> & cluster,
        int goal_clearance_cells,
        int goal_search_radius_cells
    );

    int index(
        const nav_msgs::msg::OccupancyGrid & map,
        int x,
        int y
    );
};
