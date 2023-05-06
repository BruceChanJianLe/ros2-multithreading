#ifndef ROS2_MULTITHREADING_MULTITHREAD_PUB_SUB_HPP
#define ROS2_MULTITHREADING_MULTITHREAD_PUB_SUB_HPP

#include "ros2-multithreading/constants.hpp"

// ROS2
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

// STL
#include <string>
#include <chrono>

namespace multithread
{
  class PubNode : public rclcpp::Node
  {
  public:
    explicit PubNode();
    ~PubNode();

  private:
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Logger lg_;

    int count_;

    void timerCB();
  };

  class Sub2Node : public rclcpp_lifecycle::LifecycleNode
  {
  public:
    explicit Sub2Node();
    ~Sub2Node();

  protected:
    /**
     * \brief Configure node
     *
     */
    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_configure(const rclcpp_lifecycle::State &state) override;

    /**
     * \brief Activate node
     *
     */
    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_activate(const rclcpp_lifecycle::State &state) override;

    /**
     * \brief Deactivate node
     *
     */
    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State &state) override;

    /**
     * \brief Cleanup node
     *
     */
    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_cleanup(const rclcpp_lifecycle::State &state) override;

    /**
     * \brief Shutdown node
     *
     */
    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_shutdown(const rclcpp_lifecycle::State &state) override;

  private:
    rclcpp::CallbackGroup::SharedPtr callback_group_sub1;
    rclcpp::CallbackGroup::SharedPtr callback_group_sub2;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub1_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub2_;
    rclcpp::Logger lg_;

    void subCB(const std_msgs::msg::String::ConstSharedPtr msg);
  };

} // namespace multithread

#endif /* ROS2_MULTITHREADING_MULTITHREAD_PUB_SUB_HPP */
