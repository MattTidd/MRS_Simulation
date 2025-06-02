#ifndef MRS_BT_NODE__AWAIT_REQUEST_ACTION_HPP_
#define MRS_BT_NODE__AWAIT_REQUEST_ACTION_HPP_

#include <behaviortree_cpp/action_node.h>
#include <behaviortree_cpp/condition_node.h>
#include <behaviortree_cpp/blackboard.h>

namespace mrs_bt_node
{
class AwaitRequestAction : public BT::SyncActionNode
{
public:
    // constructor:
    AwaitRequestAction(const std::string& name, const BT::NodeConfiguration& config);

    // list the ports:
    static BT::PortsList providedPorts();

    // tick method: 
    BT::NodeStatus tick() override;
};

} // namespace mrs_bt_node

#endif // MRS_BT_NODE__AWAIT_REQUEST_ACTION_HPP_