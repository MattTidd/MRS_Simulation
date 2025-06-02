#include <mrs_bt_node/participant/request_received_condition.hpp>
#include <rclcpp/rclcpp.hpp>

namespace mrs_bt_node
{
    // constructor 
    RequestReceivedCondition::RequestReceivedCondition(const std::string& name, const BT::NodeConfiguration& config) 
    : BT::ConditionNode(name, config)
    {

    }

    // port listing:
    BT::PortsList RequestReceivedCondition::providedPorts()
    {
        return{};
    }

    // tick method:
    BT::NodeStatus RequestReceivedCondition::tick()
    {
        std::cout << "Checking if Request Received..." << std::endl;
        return BT::NodeStatus::SUCCESS;
    }

} // namespace mrs_bt_node