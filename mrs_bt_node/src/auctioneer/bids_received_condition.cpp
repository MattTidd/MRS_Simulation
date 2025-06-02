#include <mrs_bt_node/auctioneer/bids_received_condition.hpp>
#include <rclcpp/rclcpp.hpp>

namespace mrs_bt_node
{
    // constructor:
    BidsReceivedCondition::BidsReceivedCondition(const std::string& name, const BT::NodeConfiguration& config)
    : BT::ConditionNode(name, config)
    {

    }

    // port listing:
    BT::PortsList BidsReceivedCondition::providedPorts()
    {
        return{};
    }

    // tick method:
    BT::NodeStatus BidsReceivedCondition::tick()
    {
        std::cout << "All Bids Received!" << std::endl;
        return BT::NodeStatus::SUCCESS;
    }
} // namespace mrs_bt_node