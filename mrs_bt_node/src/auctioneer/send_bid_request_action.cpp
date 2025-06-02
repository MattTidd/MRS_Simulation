#include <mrs_bt_node/auctioneer/send_bid_request_action.hpp>
#include <rclcpp/rclcpp.hpp>

namespace mrs_bt_node
{
    // constructor:
    SendBidRequestAction::SendBidRequestAction(const std::string& name, const BT::NodeConfiguration& config) 
    : BT::SyncActionNode(name, config)
    {

    }

    // port listing:
    BT::PortsList SendBidRequestAction::providedPorts()
    {
        return{};
    }

    // tick method:
    BT::NodeStatus SendBidRequestAction::tick()
    {
        std::cout << "Bid Request Sent" << std::endl;
        return BT::NodeStatus::SUCCESS;
    }

} // namespace mrs_bt_node
