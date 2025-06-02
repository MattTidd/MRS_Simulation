#include <mrs_bt_node/participant/await_request_action.hpp>
#include <rclcpp/rclcpp.hpp>

namespace mrs_bt_node
{
    // constructor:
    AwaitRequestAction::AwaitRequestAction(const std::string& name, const BT::NodeConfiguration& config)
    : BT::SyncActionNode(name, config)
    {

    }

    // port listing:
    BT::PortsList AwaitRequestAction::providedPorts()
    {
        return{};
    }

    // tick method:
    BT::NodeStatus AwaitRequestAction::tick()
    {
        std::cout << "Awaiting Bid Request..." << std::endl;
        return BT::NodeStatus::SUCCESS;
    }

} // namespace mrs_bt_node