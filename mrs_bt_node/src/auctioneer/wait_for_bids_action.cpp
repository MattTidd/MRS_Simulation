#include <mrs_bt_node/auctioneer/wait_for_bids_action.hpp>
#include <rclcpp/rclcpp.hpp>

namespace mrs_bt_node
{
    // constructor:
    WaitForBidsAction::WaitForBidsAction(const std::string& name, const BT::NodeConfiguration& config)
    : BT::SyncActionNode(name, config)
    {

    }

    // port listing:
    BT::PortsList WaitForBidsAction::providedPorts()
    {
        return{};
    }

    // tick method:
    BT::NodeStatus WaitForBidsAction::tick()
    {
        std::cout << "Waiting For Bids...." << std::endl;
        return BT::NodeStatus::SUCCESS;
    }
    
} // namespace mrs_bt_node