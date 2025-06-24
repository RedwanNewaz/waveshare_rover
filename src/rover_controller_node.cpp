#include "rclcpp/rclcpp.hpp"
#include "behaviortree_cpp/bt_factory.h"
#include "geometry_msgs/msg/twist.hpp"
#include "bt_nodes.cpp"
#include "rover_controller/shared_memory.hpp"

using namespace std::chrono_literals;

/* This example creates a subclass of Node and uses std::bind() to register a
* member function as a callback from the timer. */

class RoverController : public rclcpp::Node
{
  public:
    RoverController()
    : Node("RoverController")
    {
      this->declare_parameter<std::string>("tree_file", "/home/redwan/rover_ws/src/rover_controller/config/behavior_tree.xml"); // Declare tree file parameter
      this->declare_parameter<double>("wheel_base", 0.15 / 2.0); // Declare wheel base parameter

      // Create a shared data object
      double wheel_base;
      std::string tree_file;
      shared_data_ = std::make_shared<SharedTwistData>();
      
      this->get_parameter("wheel_base", wheel_base);
      this->get_parameter("tree_file", tree_file);  

      // create a BehaviorTreeFactory
      BT::BehaviorTreeFactory factory;
      factory.registerNodeType<DefaultTwist>("DefaultTwist", shared_data_);
      // factory.registerNodeType<InplaceRotation>("InplaceRotation", shared_data_);

      factory.registerSimpleCondition("InplaceRotation", [this](BT::TreeNode&) { 
        
        if (shared_data_->linear_x == 0.0 && shared_data_->angular_z != 0.0) {
          return BT::NodeStatus::SUCCESS;  // Robot is rotating in place
        }

        return BT::NodeStatus::FAILURE;
      });

      factory.registerNodeType<ReadSharedTwist>("ReadSharedTwist", shared_data_);
      factory.registerNodeType<CreateJsonCmd>("CreateJsonCmd", wheel_base);

      tree_ = std::make_shared<BT::Tree>(factory.createTreeFromFile(tree_file));

        // Create a subscription to the "cmd_vel" topic
        // The callback will be called whenever a new message is received
      subscription_ = this->create_subscription<geometry_msgs::msg::Twist>(
      "cmd_vel", 10, [this](const geometry_msgs::msg::Twist::ConstSharedPtr msg) {
          std::lock_guard<std::mutex> lock(shared_data_->mtx);
          shared_data_->linear_x = msg->linear.x;
          shared_data_->angular_z = msg->angular.z;
          shared_data_->updated = true;
      });

        // Create a timer that calls the timer_callback function every 10ms
        // Adjust the duration as needed
      timer_ = this->create_wall_timer(100ms, [this]() { this->timer_callback(); });
    }

  private:
    void timer_callback()
    {
        tree_->tickOnce();
    }

    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr subscription_;
    std::shared_ptr<SharedTwistData> shared_data_;
    std::shared_ptr<BT::Tree> tree_;
 
};



int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<RoverController>());
    rclcpp::shutdown();

    return 0;
}
