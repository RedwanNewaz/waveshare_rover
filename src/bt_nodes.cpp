#include "behaviortree_cpp/bt_factory.h"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "rover_controller/shared_memory.hpp"
#include <sstream>
#include <memory>
#include <nlohmann/json.hpp>  // you can use any JSON lib
#include <libserial/SerialStream.h>

using json = nlohmann::json;

typedef std::shared_ptr<LibSerial::SerialStream> SerialStreamPtr;


class OpenSerialPort: public BT::SyncActionNode
{
public:
    OpenSerialPort(const std::string& name, const BT::NodeConfig& config, const SerialStreamPtr& serial_port)
        : BT::SyncActionNode(name, config), serial_port_(serial_port) {}

    static BT::PortsList providedPorts() {
        return {
            BT::InputPort<std::string>("port"),
        };
    }

    BT::NodeStatus tick() override
    {
        std::string port;
        if (!getInput<std::string>("port", port)) {
            throw BT::RuntimeError("missing required input [port]");
        }

       try {
            serial_port_->Open(port);
            serial_port_->SetBaudRate(LibSerial::BaudRate::BAUD_115200);
            serial_port_->SetCharacterSize(LibSerial::CharacterSize::CHAR_SIZE_8);
            serial_port_->SetParity(LibSerial::Parity::PARITY_NONE);
            serial_port_->SetStopBits(LibSerial::StopBits::STOP_BITS_1);
            serial_port_->SetFlowControl(LibSerial::FlowControl::FLOW_CONTROL_NONE);
            serial_port_->SetRTS(false);
            serial_port_->SetDTR(false);
            return BT::NodeStatus::SUCCESS;
        } catch (const LibSerial::OpenFailed& e) {
            std::cerr << "Error opening serial port: " << e.what() << std::endl;
            return BT::NodeStatus::FAILURE;
        }      
    }
private:
    SerialStreamPtr serial_port_;
};



class SendSerialPort: public BT::SyncActionNode
{
public:
    SendSerialPort(const std::string& name, const BT::NodeConfig& config, const SerialStreamPtr& serial_port)
        : BT::SyncActionNode(name, config), serial_port_(serial_port) {}

    static BT::PortsList providedPorts() {
        return {
            BT::InputPort<std::string>("json_str"),
        };
    }

    BT::NodeStatus tick() override
    {
        std::string json_str;
        if (!getInput<std::string>("json_str", json_str)) {
            throw BT::RuntimeError("[SendSerialPort] missing required input [json_str]");
        }

       try {
            *serial_port_ << json_str << "\n";  // Send the JSON string followed by a newline
            return BT::NodeStatus::SUCCESS;
        } catch (const LibSerial::NotOpen& e) {
            std::cerr << "Serial port not open: " << e.what() << std::endl;
            return BT::NodeStatus::FAILURE;
        } catch (const std::exception& e) {
            std::cerr << "Error writing to serial port: " << e.what() << std::endl;
            return BT::NodeStatus::FAILURE;
        }
    }
private:
    SerialStreamPtr serial_port_;
};


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
                 BT::InputPort<double>("w"),
                 BT::OutputPort<std::string>("json_str") };
    }

    BT::NodeStatus tick() override
    {
        double v, w;
        getInput("v", v);
        getInput("w", w);

        auto [v_l, v_r] = calculateWheelSpeeds(v, w);

        json cmd = { {"T", 1}, {"L", v_l}, {"R", v_r} };
        std::cout << "JSON Command: " << cmd.dump() << std::endl;
        setOutput("json_str", cmd.dump());
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

