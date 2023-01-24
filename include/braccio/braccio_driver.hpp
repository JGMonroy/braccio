#include <functional>
#include <memory>
#include <thread>
#include <time.h>
#include <chrono>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "braccio/srv/status.hpp"
#include "braccio/action/braccio_cmd.hpp"

class Braccio: public rclcpp::Node{
public:
    Braccio();
    ~Braccio();

    // some definitions to make live easier
    using BraccioAction = braccio::action::BraccioCMD;
    using GoalHandleBraccio = rclcpp_action::ServerGoalHandle<BraccioAction>;

private:
    // Srv server
    rclcpp::Service<braccio::srv::Status>::SharedPtr srv_server_;

    // Action server
    rclcpp_action::Server<braccio::action::BraccioCMD>::SharedPtr action_server_;

    // Srv Callbacks
    void process_service_request(
        const std::shared_ptr<rmw_request_id_t> request_header,
        const std::shared_ptr<braccio::srv::Status::Request> request,
        std::shared_ptr<braccio::srv::Status::Response> response);

    // Action Callbacks
    rclcpp_action::GoalResponse handle_goal(
        const rclcpp_action::GoalUUID & uuid,
        std::shared_ptr<const BraccioAction::Goal> goal);

    rclcpp_action::CancelResponse handle_cancel(
        const std::shared_ptr<GoalHandleBraccio> goal_handle);

    void handle_accepted(const std::shared_ptr<GoalHandleBraccio> goal_handle);

    void execute(const std::shared_ptr<GoalHandleBraccio> goal_handle);

    // Internal variables
    geometry_msgs::msg::Point current_position;
    std::vector<float> current_joints;
    std::string serialPortFilename;
    FILE *serPortFile;
    char front_delimiter_;
	char end_delimiter_;
};

