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
        const nav_msgs::msg::OccupancyGrid & map
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

    bool isFrontier(
        const nav_msgs::msg::OccupancyGrid & map,
        int x,
        int y
    );

    int index(
        const nav_msgs::msg::OccupancyGrid & map,
        int x,
        int y
    );
};