#include "frontier_explorer/frontier_explorer_node.hpp"

#include <chrono>

using namespace std::chrono_literals;

FrontierExplorerNode::FrontierExplorerNode()
: Node("frontier_explorer_node"),
  map_received_(false),
  goal_active_(false)
{
    loadParameters();

    map_sub_ =
        this->create_subscription<nav_msgs::msg::OccupancyGrid>(
            map_topic_,
            10,
            std::bind(
                &FrontierExplorerNode::mapCallback,
                this,
                std::placeholders::_1
            )
        );

    nav_client_ =
        rclcpp_action::create_client<NavigateToPose>(
            this,
            action_name_
        );

    timer_ =
        this->create_wall_timer(
            std::chrono::duration<double>(explore_period_),
            std::bind(
                &FrontierExplorerNode::timerCallback,
                this
            )
        );

    RCLCPP_INFO(this->get_logger(), "Frontier Explorer gestartet");
}

void FrontierExplorerNode::loadParameters()
{
    this->declare_parameter<std::string>(
        "map_topic",
        "/map"
    );

    this->declare_parameter<std::string>(
        "action_name",
        "navigate_to_pose"
    );

    this->declare_parameter<double>(
        "explore_period",
        2.0
    );

    map_topic_ =
        this->get_parameter("map_topic").as_string();

    action_name_ =
        this->get_parameter("action_name").as_string();

    explore_period_ =
        this->get_parameter("explore_period").as_double();
}

void FrontierExplorerNode::mapCallback(
    const nav_msgs::msg::OccupancyGrid::SharedPtr msg
)
{
    current_map_ = *msg;
    map_received_ = true;
}

void FrontierExplorerNode::timerCallback()
{
    if (!map_received_)
    {
        RCLCPP_WARN_THROTTLE(
            this->get_logger(),
            *this->get_clock(),
            3000,
            "Noch keine Map erhalten"
        );

        return;
    }

    if (goal_active_)
    {
        return;
    }

    auto frontier =
        explorer_.findFrontier(current_map_);

    if (!frontier.has_value())
    {
        RCLCPP_INFO(
            this->get_logger(),
            "Keine Frontier mehr gefunden"
        );

        return;
    }

    auto goal =
        explorer_.mapToPose(
            current_map_,
            frontier.value()
        );

    goal.header.stamp = this->now();

    RCLCPP_INFO(
        this->get_logger(),
        "Neue Frontier gefunden: x=%.2f y=%.2f",
        goal.pose.position.x,
        goal.pose.position.y
    );

    sendGoal(goal);
}

void FrontierExplorerNode::sendGoal(
    const geometry_msgs::msg::PoseStamped & goal
)
{
    if (!nav_client_->wait_for_action_server(2s))
    {
        RCLCPP_WARN(
            this->get_logger(),
            "Nav2 Action Server nicht erreichbar"
        );

        return;
    }

    NavigateToPose::Goal nav_goal;
    nav_goal.pose = goal;

    auto options =
        rclcpp_action::Client<NavigateToPose>::SendGoalOptions();

    options.goal_response_callback =
        [this](const GoalHandleNav::SharedPtr & goal_handle)
        {
            if (!goal_handle)
            {
                RCLCPP_WARN(
                    this->get_logger(),
                    "Goal abgelehnt"
                );

                goal_active_ = false;
                return;
            }

            RCLCPP_INFO(
                this->get_logger(),
                "Goal akzeptiert"
            );

            goal_active_ = true;
        };

    options.result_callback =
        [this](const GoalHandleNav::WrappedResult & result)
        {
            goal_active_ = false;

            switch (result.code)
            {
                case rclcpp_action::ResultCode::SUCCEEDED:
                    RCLCPP_INFO(
                        this->get_logger(),
                        "Goal erreicht"
                    );
                    break;

                case rclcpp_action::ResultCode::ABORTED:
                    RCLCPP_WARN(
                        this->get_logger(),
                        "Goal abgebrochen"
                    );
                    break;

                case rclcpp_action::ResultCode::CANCELED:
                    RCLCPP_WARN(
                        this->get_logger(),
                        "Goal abgebrochen/canceled"
                    );
                    break;

                default:
                    RCLCPP_WARN(
                        this->get_logger(),
                        "Unbekanntes Result"
                    );
                    break;
            }
        };

    nav_client_->async_send_goal(nav_goal, options);
}