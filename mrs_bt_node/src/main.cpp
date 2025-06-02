#include <rclcpp/rclcpp.hpp>
#include <behaviortree_cpp/bt_factory.h>
#include <mrs_bt_node/auctioneer/send_bid_request_action.hpp>
#include <mrs_bt_node/auctioneer/bids_received_condition.hpp>
#include <mrs_bt_node/auctioneer/wait_for_bids_action.hpp>
#include <mrs_bt_node/auctioneer/allocation_action.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("mrs_bt_node");

    try {
        // initialize behaviour tree factory:
        BT::BehaviorTreeFactory factory;

        // register custom nodes:
        factory.registerNodeType<mrs_bt_node::SendBidRequestAction>("SendBidRequestAction");
        factory.registerNodeType<mrs_bt_node::BidsReceivedCondition>("BidsReceivedCondition");
        factory.registerNodeType<mrs_bt_node::WaitForBidsAction>("WaitForBidsAction");
        factory.registerNodeType<mrs_bt_node::AllocationAction>("AllocationAction");

        // get package path:
        const std::string package_share_dir = ament_index_cpp::get_package_share_directory("mrs_bt_node");

        // load bt from XML file using absolute path:
        const std::string tree_filename = node->declare_parameter<std::string>(
            "tree_file",    // try this
            package_share_dir + "/trees/auctioneer_tree.xml" // fallback to this (remove)
        );

        // debug:
        RCLCPP_INFO(node->get_logger(), "Loading tree from: %s", tree_filename.c_str());

        // create the tree:
        // blackboard stuff goes here //
        auto tree = factory.createTreeFromFile(tree_filename);

        // configure tree tick timer:
        const auto tick_period = std::chrono::milliseconds(
            node->declare_parameter<int>("tick_rate_ms", 100)
        );

        auto timer = node->create_wall_timer(tick_period, [&](){
            try {
                const BT::NodeStatus status = tree.tickOnce();
                if(status == BT::NodeStatus::RUNNING) {
                    return;
                }

                // shutdown when tree completes:
                RCLCPP_INFO(node->get_logger(), "Behaviour Tree completed with status: %d", static_cast<int>(status));
                rclcpp::shutdown();
            } catch (const std::exception& e) {
                RCLCPP_ERROR(node->get_logger(), "Error executing tree: %s", e.what());
                rclcpp::shutdown();
            }
        });

        RCLCPP_INFO(node->get_logger(), "Behavior Tree controller started!");
        rclcpp::spin(node);
    } catch (const std::exception& e) {
        RCLCPP_FATAL(node->get_logger(), "Failed to initialize: %s", e.what());
        return EXIT_FAILURE;
    }

    rclcpp::shutdown();
    return EXIT_SUCCESS;
}