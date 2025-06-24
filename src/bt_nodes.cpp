#include "behaviortree_cpp/bt_factory.h"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "rover_controller/shared_memory.hpp"
#include <sstream>
#include <memory>
#include <nlohmann/json.hpp>  // you can use any JSON lib

using json = nlohmann::json;


class DefaultTwist : public BT::SyncActionNode
{
public:
    DefaultTwist(const std::string& name, const BT::NodeConfig& config, const SharedTwistDataPtr& shared_data)
        : BT::SyncActionNode(name, config), shared_data_(shared_data) {}

    static BT::PortsList providedPorts() {
        return {BT::InputPort<double>("v_in"),
                BT::InputPort<double>("w_in"), 
                BT::OutputPort<double>("v"),
                BT::OutputPort<double>("w") };
    }

    BT::NodeStatus tick() override
    {
        double v_in, w_in;
        std::lock_guard<std::mutex> lock(shared_data_->mtx);
        // Get the input velocities
        getInput("v_in", v_in);
        getInput("w_in", w_in);
        // Set default values for linear and angular velocities
        setOutput("v", v_in);
        setOutput("w", w_in);

        // Update the shared data flag
        shared_data_->updated = false;
        return BT::NodeStatus::SUCCESS;
    }

private:
    SharedTwistDataPtr shared_data_;  // Pointer to shared data, if needed
};

class ReadSharedTwist : public BT::SyncActionNode
{
public:
    ReadSharedTwist(const std::string& name, const BT::NodeConfig& config,
                    const SharedTwistDataPtr& shared_data)
        : BT::SyncActionNode(name, config), shared_data_(shared_data) {}

    static BT::PortsList providedPorts() {
        return { BT::OutputPort<double>("v"),
                 BT::OutputPort<double>("w") };
    }

    BT::NodeStatus tick() override
    {
        std::lock_guard<std::mutex> lock(shared_data_->mtx);
        if(!shared_data_->updated)
        {
            return BT::NodeStatus::FAILURE;  // No new data to read
        }
        // Read the shared data
        setOutput("v", shared_data_->linear_x);
        setOutput("w", shared_data_->angular_z);
        shared_data_->updated = false;  // Reset the updated flag

        return BT::NodeStatus::SUCCESS;
    }

private:
    SharedTwistDataPtr shared_data_;
};

class CreateJsonCmd : public BT::SyncActionNode
{
public:
    CreateJsonCmd(const std::string& name, const BT::NodeConfig& config,  double wheel_base)
        : BT::SyncActionNode(name, config), wheel_base_(wheel_base) {}

    static BT::PortsList providedPorts() {
        return { BT::InputPort<double>("v"),
                 BT::InputPort<double>("w")};
    }

    BT::NodeStatus tick() override
    {
        double v, w;
        getInput("v", v);
        getInput("w", w);

        auto [v_l, v_r] = calculateWheelSpeeds(v, w);

        json cmd = { {"T", 1}, {"L", v_l}, {"R", v_r} };
        std::cout << "JSON Command: " << cmd.dump() << std::endl;
        return BT::NodeStatus::SUCCESS;
    }

private:
    double wheel_base_;  // Example wheel base, adjust as needed

    std::pair<double, double> calculateWheelSpeeds(double v, double w)
    {
        double v_r = v + (wheel_base_ / 2.0) * w;
        double v_l = v - (wheel_base_ / 2.0) * w;
        return {v_l, v_r};
    }

};

