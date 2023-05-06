#include "ros2-multithreading/multithread_pub_sub.hpp"

namespace multithread
{
  PubNode::PubNode()
  : rclcpp::Node(util::PUB_NODE_NAME, rclcpp::NodeOptions().use_intra_process_comms(false))
  , lg_{this->get_logger()}
  , count_{0}
  {
    pub_ = this->create_publisher<std_msgs::msg::String>(util::TOPIC_NAME, 1);
    timer_ = this->create_wall_timer(std::chrono::milliseconds(1000),
      [this]() { this->timerCB(); }
    );
  }

  PubNode::~PubNode()
  {
  }

  void PubNode::timerCB()
  {
    auto msg = std_msgs::msg::String();
    msg.data = "Hello World! " + std::to_string(this->count_++);

    // Extract current thread
    auto string_thread_id = std::to_string(std::hash<std::thread::id>()(std::this_thread::get_id()));

    // Prep display message
    RCLCPP_INFO_STREAM(this->lg_, "THREAD " << string_thread_id << " has spoken '" << msg.data.c_str() << "'");

    this->pub_->publish(msg);
  }

  Sub2Node::Sub2Node()
  : rclcpp_lifecycle::LifecycleNode(util::SUB_NODE_NAME, rclcpp::NodeOptions().use_intra_process_comms(false))
  , lg_{this->get_logger()}
  {
  }

  Sub2Node::~Sub2Node()
  {
  }

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn Sub2Node::on_configure(const rclcpp_lifecycle::State &state)
  {
    /* These define the callback groups
     * They don't really do much on their own, but they have to exist in order to
     * assign callbacks to them. They're also what the executor looks for when trying to run multiple threads
     */
    callback_group_sub1 = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    callback_group_sub2 = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
  }

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn Sub2Node::on_activate(const rclcpp_lifecycle::State &state)
  {
    // Each of these callback groups is basically a thread
    // Everything assigned to one of them gets bundled into the same thread
    auto sub1_opt = rclcpp::SubscriptionOptions();
    sub1_opt.callback_group = callback_group_sub1;
    auto sub2_opt = rclcpp::SubscriptionOptions();
    sub2_opt.callback_group = callback_group_sub2;

    sub1_ = this->create_subscription<std_msgs::msg::String>(
        util::TOPIC_NAME,
        rclcpp::QoS(1),
        [this](const std_msgs::msg::String::ConstSharedPtr msg)
        {
          this->subCB(msg);
        },
        // This is where we set the callback group.
        // This subscription will run with callback group subscriber1
        sub1_opt
    );

    sub2_ = this->create_subscription<std_msgs::msg::String>(
        util::TOPIC_NAME,
        rclcpp::QoS(1),
        [this](const std_msgs::msg::String::ConstSharedPtr msg)
        {
          this->subCB(msg);
        },
        // This is where we set the callback group.
        // This subscription will run with callback group subscriber1
        sub2_opt
    );

    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
  }

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn Sub2Node::on_deactivate(const rclcpp_lifecycle::State &state)
  {
    sub1_.reset();
    sub2_.reset();
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
  }

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn Sub2Node::on_cleanup(const rclcpp_lifecycle::State &state)
  {
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
  }

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn Sub2Node::on_shutdown(const rclcpp_lifecycle::State &state)
  {
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
  }


  void Sub2Node::subCB(const std_msgs::msg::String::ConstSharedPtr msg)
  {
    auto string_thread_id = std::to_string(std::hash<std::thread::id>()(std::this_thread::get_id()));

    // Prep display message
    RCLCPP_INFO_STREAM(lg_, "THREAD " << string_thread_id << " => Heard '" << msg->data.c_str() << "'");
  }
} // namespace multithread