#include "frontier_explorer/frontier_explorer.hpp"

#include <cmath>
#include <limits>
#include <queue>

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
    if (!isInBounds(map, x, y))
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
    if (!isInBounds(map, x, y))
    {
        return false;
    }

    return map.data[index(map, x, y)] == -1;
}

bool FrontierExplorer::isInBounds(
    const nav_msgs::msg::OccupancyGrid & map,
    int x,
    int y
)
{
    return x >= 0 && y >= 0 &&
        x < static_cast<int>(map.info.width) &&
        y < static_cast<int>(map.info.height);
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

bool FrontierExplorer::hasClearance(
    const nav_msgs::msg::OccupancyGrid & map,
    int x,
    int y,
    int radius_cells
)
{
    for (int dy = -radius_cells; dy <= radius_cells; dy++)
    {
        for (int dx = -radius_cells; dx <= radius_cells; dx++)
        {
            if ((dx * dx + dy * dy) > (radius_cells * radius_cells))
            {
                continue;
            }

            if (!isFree(map, x + dx, y + dy))
            {
                return false;
            }
        }
    }

    return true;
}

std::optional<FrontierPoint> FrontierExplorer::findSafeGoalNear(
    const nav_msgs::msg::OccupancyGrid & map,
    const std::vector<FrontierPoint> & cluster,
    int goal_clearance_cells,
    int goal_search_radius_cells
)
{
    if (cluster.empty())
    {
        return std::nullopt;
    }

    double centroid_x = 0.0;
    double centroid_y = 0.0;
    for (const auto & point : cluster)
    {
        centroid_x += point.x;
        centroid_y += point.y;
    }

    centroid_x /= static_cast<double>(cluster.size());
    centroid_y /= static_cast<double>(cluster.size());

    const int center_x = static_cast<int>(std::lround(centroid_x));
    const int center_y = static_cast<int>(std::lround(centroid_y));
    double best_score = std::numeric_limits<double>::max();
    std::optional<FrontierPoint> best_goal;

    for (int dy = -goal_search_radius_cells; dy <= goal_search_radius_cells; dy++)
    {
        for (int dx = -goal_search_radius_cells; dx <= goal_search_radius_cells; dx++)
        {
            const int x = center_x + dx;
            const int y = center_y + dy;
            if (!hasClearance(map, x, y, goal_clearance_cells))
            {
                continue;
            }

            const double dist_to_frontier =
                std::hypot(
                    static_cast<double>(x) - centroid_x,
                    static_cast<double>(y) - centroid_y
                );

            if (dist_to_frontier < best_score)
            {
                best_score = dist_to_frontier;
                best_goal = FrontierPoint{x, y};
            }
        }
    }

    return best_goal;
}

std::optional<FrontierPoint> FrontierExplorer::findFrontier(
    const nav_msgs::msg::OccupancyGrid & map,
    int goal_clearance_cells,
    int goal_search_radius_cells,
    int min_frontier_cluster_size
)
{
    const int width = static_cast<int>(map.info.width);
    const int height = static_cast<int>(map.info.height);
    std::vector<bool> visited(map.data.size(), false);
    std::optional<FrontierPoint> best_goal;
    std::size_t best_cluster_size = 0;

    for (int y = 1; y < static_cast<int>(map.info.height) - 1; y++)
    {
        for (int x = 1; x < static_cast<int>(map.info.width) - 1; x++)
        {
            const int start_index = index(map, x, y);
            if (visited[start_index] || !isFrontier(map, x, y))
            {
                continue;
            }

            std::vector<FrontierPoint> cluster;
            std::queue<FrontierPoint> queue;
            visited[start_index] = true;
            queue.push(FrontierPoint{x, y});

            while (!queue.empty())
            {
                const auto current = queue.front();
                queue.pop();
                cluster.push_back(current);

                for (int dy = -1; dy <= 1; dy++)
                {
                    for (int dx = -1; dx <= 1; dx++)
                    {
                        if (dx == 0 && dy == 0)
                        {
                            continue;
                        }

                        const int nx = current.x + dx;
                        const int ny = current.y + dy;
                        if (nx <= 0 || ny <= 0 || nx >= width - 1 || ny >= height - 1)
                        {
                            continue;
                        }

                        const int next_index = index(map, nx, ny);
                        if (visited[next_index] || !isFrontier(map, nx, ny))
                        {
                            continue;
                        }

                        visited[next_index] = true;
                        queue.push(FrontierPoint{nx, ny});
                    }
                }
            }

            if (cluster.size() < static_cast<std::size_t>(min_frontier_cluster_size))
            {
                continue;
            }

            auto safe_goal =
                findSafeGoalNear(
                    map,
                    cluster,
                    goal_clearance_cells,
                    goal_search_radius_cells
                );

            if (safe_goal.has_value() && cluster.size() > best_cluster_size)
            {
                best_cluster_size = cluster.size();
                best_goal = safe_goal;
            }
        }
    }

    return best_goal;
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
