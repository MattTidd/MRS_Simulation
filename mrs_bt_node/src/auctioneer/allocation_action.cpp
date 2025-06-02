#include <mrs_bt_node/auctioneer/allocation_action.hpp>
#include <rclcpp/rclcpp.hpp>

namespace mrs_bt_node
{
    // constructor:
    AllocationAction::AllocationAction(const std::string& name, const BT::NodeConfiguration& config) 
    : BT::SyncActionNode(name, config)
    {

    }

    // port listing:
    BT::PortsList AllocationAction::providedPorts()
    {
        return{};
    }

    // tick method:
    BT::NodeStatus AllocationAction::tick()
    {
        std::cout << "Task Allocated!" << std::endl;
        return BT::NodeStatus::SUCCESS;
    }

} // namespace mrs_bt_node
