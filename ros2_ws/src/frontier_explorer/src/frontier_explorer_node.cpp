#include "frontier_explorer/frontier_explorer_node.hpp"

#include <chrono>

using namespace std::chrono_literals;

FrontierExplorerNode::FrontierExplorerNode()
: Node("frontier_explorer_node"),
  map_received_(false),
  goal_active_(false),
  finish_requested_(false),
  no_frontier_count_(0)
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

    finish_mapping_client_ =
        this->create_client<std_srvs::srv::Trigger>(
            finish_mapping_service_
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

    this->declare_parameter<bool>(
        "auto_finish_enabled",
        true
    );

    this->declare_parameter<int>(
        "no_frontier_finish_count",
        5
    );

    this->declare_parameter<double>(
        "completion_coverage_ratio",
        0.99
    );

    this->declare_parameter<double>(
        "completion_hold_time",
        15.0
    );

    this->declare_parameter<std::string>(
        "finish_mapping_service",
        "/task_manager/finish_mapping"
    );

    map_topic_ =
        this->get_parameter("map_topic").as_string();

    action_name_ =
        this->get_parameter("action_name").as_string();

    explore_period_ =
        this->get_parameter("explore_period").as_double();

    auto_finish_enabled_ =
        this->get_parameter("auto_finish_enabled").as_bool();

    no_frontier_finish_count_ =
        this->get_parameter("no_frontier_finish_count").as_int();

    completion_coverage_ratio_ =
        this->get_parameter("completion_coverage_ratio").as_double();

    completion_hold_time_ =
        this->get_parameter("completion_hold_time").as_double();

    finish_mapping_service_ =
        this->get_parameter("finish_mapping_service").as_string();
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

    if (
        auto_finish_enabled_ &&
        !finish_requested_ &&
        shouldFinishByCoverage()
    )
    {
        requestMappingFinish();
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
        no_frontier_count_++;

        RCLCPP_INFO(
            this->get_logger(),
            "Keine Frontier gefunden (%d/%d)",
            no_frontier_count_,
            no_frontier_finish_count_
        );

        if (
            auto_finish_enabled_ &&
            !finish_requested_ &&
            no_frontier_count_ >= no_frontier_finish_count_
        )
        {
            requestMappingFinish();
        }

        return;
    }

    no_frontier_count_ = 0;

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

double FrontierExplorerNode::calculateCoverageRatio(
    const nav_msgs::msg::OccupancyGrid & map
) const
{
    if (map.data.empty())
    {
        return 0.0;
    }

    std::size_t known_cells = 0;
    for (const auto cell : map.data)
    {
        if (cell != -1)
        {
            known_cells++;
        }
    }

    return static_cast<double>(known_cells) / static_cast<double>(map.data.size());
}

bool FrontierExplorerNode::shouldFinishByCoverage()
{
    const double coverage_ratio =
        calculateCoverageRatio(current_map_);

    if (coverage_ratio < completion_coverage_ratio_)
    {
        coverage_threshold_since_.reset();
        return false;
    }

    const auto now = this->now();
    if (!coverage_threshold_since_.has_value())
    {
        coverage_threshold_since_ = now;
        RCLCPP_INFO(
            this->get_logger(),
            "Map-Abdeckung %.1f%% erreicht; warte %.1f s vor Abschluss",
            coverage_ratio * 100.0,
            completion_hold_time_
        );
        return false;
    }

    const double held_seconds =
        (now - coverage_threshold_since_.value()).seconds();

    if (held_seconds < completion_hold_time_)
    {
        return false;
    }

    RCLCPP_INFO(
        this->get_logger(),
        "Map-Abdeckung %.1f%% seit %.1f s stabil; Mapping wird abgeschlossen",
        coverage_ratio * 100.0,
        held_seconds
    );

    return true;
}

void FrontierExplorerNode::requestMappingFinish()
{
    finish_requested_ = true;

    if (!finish_mapping_client_->wait_for_service(2s))
    {
        RCLCPP_ERROR(
            this->get_logger(),
            "Taskmanager-Service %s nicht erreichbar; Mapping wird nicht automatisch abgeschlossen",
            finish_mapping_service_.c_str()
        );

        finish_requested_ = false;
        return;
    }

    auto request = std::make_shared<std_srvs::srv::Trigger::Request>();

    RCLCPP_INFO(
        this->get_logger(),
        "Map scheint vollstaendig erkundet; starte Abschlusssequenz ueber Taskmanager"
    );

    finish_mapping_client_->async_send_request(request);
}
