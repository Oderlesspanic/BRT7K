#include "hardware_supervisor/hardware_supervisor_node.hpp"

#include <chrono>
#include <memory>

#include "hardware_supervisor/composite_device_monitor.hpp"
#include "hardware_supervisor/usb_monitor.hpp"
#include "hardware_supervisor/i2c_monitor.hpp"
#include "hardware_supervisor/topic_monitor.hpp"
#include "hardware_supervisor/esp32_heartbeat_monitor.hpp"

HardwareSupervisorNode::HardwareSupervisorNode()
: Node("hardware_supervisor")
{
  load_config();
  create_monitors();

  status_pub_ =
    create_publisher<diagnostic_msgs::msg::DiagnosticArray>(
      "/diagnostics",
      10);

  const auto period =
    std::chrono::duration<double>(1.0 / update_rate_hz_);

  timer_ = create_wall_timer(
    std::chrono::duration_cast<std::chrono::milliseconds>(period),
    std::bind(&HardwareSupervisorNode::publish_status, this));
    RCLCPP_INFO(get_logger(), "Hardware Supervisor Node started with update rate %.2f Hz", update_rate_hz_);
}

void HardwareSupervisorNode::load_config()
{
  declare_parameter<std::string>("hardware_id", "mobile_robot");
  declare_parameter<double>("update_rate_hz", 2.0);
  declare_parameter<std::vector<std::string>>("device_names", std::vector<std::string>{});

  get_parameter("hardware_id", hardware_id_);
  get_parameter("update_rate_hz", update_rate_hz_);

  std::vector<std::string> names;
  get_parameter("device_names", names);

  for (const auto & name : names) {
    DeviceConfig config;
    config.name = name;

    declare_parameter<std::string>("devices." + name + ".physical_type", "none");
    declare_parameter<std::string>("devices." + name + ".communication_type", "topic");
    declare_parameter<std::string>("devices." + name + ".path", "");
    declare_parameter<std::string>("devices." + name + ".topic", "");
    declare_parameter<std::string>("devices." + name + ".topic_type", "");
    declare_parameter<double>("devices." + name + ".timeout", 1.0);
    declare_parameter<int>("devices." + name + ".i2c_bus", -1);
    declare_parameter<int>("devices." + name + ".i2c_address", -1);

    get_parameter("devices." + name + ".physical_type", config.physical_type);
    get_parameter("devices." + name + ".communication_type", config.communication_type);
    get_parameter("devices." + name + ".path", config.path);
    get_parameter("devices." + name + ".topic", config.topic);
    get_parameter("devices." + name + ".topic_type", config.topic_type);
    get_parameter("devices." + name + ".timeout", config.timeout);
    get_parameter("devices." + name + ".i2c_bus", config.i2c_bus);
    get_parameter("devices." + name + ".i2c_address", config.i2c_address);

    device_configs_.push_back(config);
  }
}

void HardwareSupervisorNode::create_monitors()
{
  for (const auto & config : device_configs_) {
    auto device = std::make_shared<CompositeDeviceMonitor>(config.name);

    if (config.physical_type == "usb") {
      device->add_monitor(
        std::make_shared<UsbMonitor>(
          config.name + "_usb",
          config.path));
    } else if (config.physical_type == "i2c") {
      device->add_monitor(
        std::make_shared<I2CMonitor>(
          config.name + "_i2c",
          config.i2c_bus,
          config.i2c_address));
    } else if (config.physical_type == "none") {
      // no physical monitor
    } else {
      RCLCPP_WARN(
        get_logger(),
        "Unknown physical_type '%s' for device '%s'",
        config.physical_type.c_str(),
        config.name.c_str());
    }

    if (config.communication_type == "esp32_heartbeat") {
      device->add_monitor(
        std::make_shared<ESP32HeartbeatMonitor>(
          this,
          config.name + "_heartbeat",
          config.path,
          config.topic,
          config.timeout));
    } else if (config.communication_type == "topic") {
      device->add_monitor(
        std::make_shared<TopicMonitor>(
          this,
          config.name + "_topic",
          config.topic,
          config.topic_type,
          config.timeout));
    } else {
      RCLCPP_WARN(
        get_logger(),
        "Unknown communication_type '%s' for device '%s'",
        config.communication_type.c_str(),
        config.name.c_str());
    }

    supervisor_.add_monitor(device);
  }
  RCLCPP_INFO(get_logger(), "Created monitors for %zu devices", device_configs_.size());
}

void HardwareSupervisorNode::publish_status()
{
  auto msg = supervisor_.build_status_msg(
    now(),
    hardware_id_);

  status_pub_->publish(msg);
}