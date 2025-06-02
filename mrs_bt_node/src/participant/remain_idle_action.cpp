#include <mrs_bt_node/participant/remain_idle_action.hpp>
#include <rclcpp/rclcpp.hpp>

namespace mrs_bt_node
{
    // constructor 
    RemainIdleAction::RemainIdleAction(const std::string& name, const BT::NodeConfiguration& config) 
    : BT::SyncActionNode(name, config)
    {

    }

    // port listing:
    BT::PortsList RemainIdleAction::providedPorts()
    {
        return{};
    }

    // tick method:
    BT::NodeStatus RemainIdleAction::tick()
    {
        std::cout << 'Remaining Idle...' << std::endl;
        return BT::NodeStatus::SUCCESS;
    }

} // namespace mrs_bt_node