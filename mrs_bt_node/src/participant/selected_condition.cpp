#include <mrs_bt_node/participant/selected_condition.hpp>
#include <rclcpp/rclcpp.hpp>

namespace mrs_bt_node
{
    // constructor 
    SelectedCondition::SelectedCondition(const std::string& name, const BT::NodeConfiguration& config) 
    : BT::ConditionNode(name, config)
    {

    }

    // port listing:
    BT::PortsList SelectedCondition::providedPorts()
    {
        return{};
    }

    // tick method:
    BT::NodeStatus SelectedCondition::tick()
    {
        std::cout << "Selected!" << std::endl;
        return BT::NodeStatus::SUCCESS;
    }

} // namespace mrs_bt_node