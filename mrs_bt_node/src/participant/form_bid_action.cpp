#include <mrs_bt_node/participant/form_bid_action.hpp>
#include <rclcpp/rclcpp.hpp>

namespace mrs_bt_node
{
    // constructor 
    FormBidAction::FormBidAction(const std::string& name, const BT::NodeConfiguration& config) 
    : BT::SyncActionNode(name, config)
    {

    }

    // port listing:
    BT::PortsList FormBidAction::providedPorts()
    {
        return{};
    }

    // tick method:
    BT::NodeStatus FormBidAction::tick()
    {
        std::cout << "Formulating Bid..." << std::endl;
        return BT::NodeStatus::SUCCESS;
    }

} // namespace mrs_bt_node