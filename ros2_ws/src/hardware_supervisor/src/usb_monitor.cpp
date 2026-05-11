#include "hardware_supervisor/usb_monitor.hpp"

#include <filesystem>

UsbMonitor::UsbMonitor(const std::string & name, const std::string & path)
: name_(name), path_(path)
{
}

std::string UsbMonitor::name() const
{
  return name_;
}

bool UsbMonitor::is_present() const
{
  return std::filesystem::exists(path_);
}

bool UsbMonitor::is_alive() const
{
  return is_present();
}

std::string UsbMonitor::message() const
{
  if (is_present()) {
    return "USB device present";
  }

  return std::string("USB device missing: ") + path_.string();
}