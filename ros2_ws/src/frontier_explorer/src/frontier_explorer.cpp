#include "frontier_explorer/frontier_explorer.hpp"

FrontierExplorer::FrontierExplorer()
{
}

int FrontierExplorer::index(
    const nav_msgs::msg::OccupancyGrid & map,
    int x,
    int y
)
{
    return y * map.info.width + x;
}

bool FrontierExplorer::isFree(
    const nav_msgs::msg::OccupancyGrid & map,
    int x,
    int y
)
{
    if (x < 0 || y < 0 ||
        x >= static_cast<int>(map.info.width) ||
        y >= static_cast<int>(map.info.height))
    {
        return false;
    }

    return map.data[index(map, x, y)] == 0;
}

bool FrontierExplorer::isUnknown(
    const nav_msgs::msg::OccupancyGrid & map,
    int x,
    int y
)
{
    if (x < 0 || y < 0 ||
        x >= static_cast<int>(map.info.width) ||
        y >= static_cast<int>(map.info.height))
    {
        return false;
    }

    return map.data[index(map, x, y)] == -1;
}

bool FrontierExplorer::isFrontier(
    const nav_msgs::msg::OccupancyGrid & map,
    int x,
    int y
)
{
    if (!isFree(map, x, y))
    {
        return false;
    }

    for (int dy = -1; dy <= 1; dy++)
    {
        for (int dx = -1; dx <= 1; dx++)
        {
            if (dx == 0 && dy == 0)
            {
                continue;
            }

            if (isUnknown(map, x + dx, y + dy))
            {
                return true;
            }
        }
    }

    return false;
}

std::optional<FrontierPoint> FrontierExplorer::findFrontier(
    const nav_msgs::msg::OccupancyGrid & map
)
{
    for (int y = 1; y < static_cast<int>(map.info.height) - 1; y++)
    {
        for (int x = 1; x < static_cast<int>(map.info.width) - 1; x++)
        {
            if (isFrontier(map, x, y))
            {
                FrontierPoint point;
                point.x = x;
                point.y = y;

                return point;
            }
        }
    }

    return std::nullopt;
}

geometry_msgs::msg::PoseStamped FrontierExplorer::mapToPose(
    const nav_msgs::msg::OccupancyGrid & map,
    const FrontierPoint & point
)
{
    geometry_msgs::msg::PoseStamped pose;

    pose.header.frame_id = "map";

    double resolution = map.info.resolution;

    double origin_x = map.info.origin.position.x;
    double origin_y = map.info.origin.position.y;

    pose.pose.position.x =
        origin_x + (point.x + 0.5) * resolution;

    pose.pose.position.y =
        origin_y + (point.y + 0.5) * resolution;

    pose.pose.position.z = 0.0;

    pose.pose.orientation.w = 1.0;

    return pose;
}