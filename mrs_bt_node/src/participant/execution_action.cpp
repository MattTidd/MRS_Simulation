#include <mrs_bt_node/participant/execution_action.hpp>
#include <rclcpp/rclcpp.hpp>

namespace mrs_bt_node
{
    // constructor:
    ExecutionAction::ExecutionAction(const std::string& name, const BT::NodeConfiguration& config)
    : BT::SyncActionNode(name, config)
    {

    }

    // port listing:
    BT::PortsList ExecutionAction::providedPorts()
    {
        return{};
    }

    // tick method:
    BT::NodeStatus ExecutionAction::tick()
    {
        std::cout << "Executing Task!" << std::endl;
        return BT::NodeStatus::SUCCESS;
    }

} // namespace mrs_bt_node