#include "rclcpp/rclcpp.hpp"
#include "shr_msgs/msg/world_state.hpp"
#include <memory>
#include "tf2_ros/buffer.h"
#include <std_msgs/msg/int32.hpp>
#include <std_msgs/msg/string.hpp>
#include <tf2_ros/transform_listener.h>
#include <shr_parameters.hpp>
#include "gui_interfaces/srv/action_req.hpp"

#pragma once

class WorldStateListener : public rclcpp::Node {
private:
    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr eating_sub_;
    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr charging_sub_;
    rclcpp::Subscription<builtin_interfaces::msg::Time>::SharedPtr time_sub_;
    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr taking_medicine_sub_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr person_at_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr robot_at_;
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    std::shared_ptr<shr_msgs::msg::WorldState> world_state_;
    std::mutex tf_buffer_mtx;
    std::mutex terminate_mtx;
    std::mutex world_state_mtx;
    bool terminate_node_;
    std::shared_ptr<shr_parameters::ParamListener> param_listener_;
    rclcpp::Client<gui_interfaces::srv::ActionReq>::SharedPtr client_;

public:
    std::string person_at;
    std::string robot_at;

    WorldStateListener(const std::string &node_name,  std::shared_ptr<shr_parameters::ParamListener> param_listener)
            : rclcpp::Node(
            node_name) {
        terminate_node_ = false;
        world_state_ = std::make_shared<shr_msgs::msg::WorldState>();
        param_listener_ = param_listener;
        auto params = param_listener->get_params();

        tf_buffer_ = std::make_unique<tf2_ros::Buffer>(get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_, true);

        eating_sub_ = create_subscription<std_msgs::msg::Int32>(
                params.topics.person_eating, 10, [this](const std_msgs::msg::Int32::SharedPtr msg) {
                    std::lock_guard<std::mutex> lock(world_state_mtx);
                    world_state_->person_eating = msg->data;
                });
        taking_medicine_sub_ = create_subscription<std_msgs::msg::Int32>(
                params.topics.person_taking_medicine, 10, [this](const std_msgs::msg::Int32::SharedPtr msg) {
                    std::lock_guard<std::mutex> lock(world_state_mtx);
                    world_state_->person_taking_medicine = msg->data;
                });
        time_sub_ = create_subscription<builtin_interfaces::msg::Time>(
                params.topics.time, 10, [this](const builtin_interfaces::msg::Time::SharedPtr msg) {
                    std::lock_guard<std::mutex> lock(world_state_mtx);
                    world_state_->time = *msg;
                });
        charging_sub_ = create_subscription<std_msgs::msg::Int32>(
                params.topics.robot_charging, 10, [this](const std_msgs::msg::Int32::SharedPtr msg) {
                    std::lock_guard<std::mutex> lock(world_state_mtx);
                    world_state_->robot_charging = msg->data;
                });

        robot_at_ = create_subscription<std_msgs::msg::String>(
                "robot_at", 10, [this](const std_msgs::msg::String::SharedPtr msg) {
                    robot_at=msg->data;
                });

        person_at_ = create_subscription<std_msgs::msg::String>(
                "person_at", 10, [this](const std_msgs::msg::String::SharedPtr msg) {
                    person_at=msg->data;
                });
        client_ = this->create_client<gui_interfaces::srv::ActionReq>("input_request");
    }

    std::string send_request(const std::string &question, const std::string &location) {
        auto request = std::make_shared<gui_interfaces::srv::ActionReq::Request>();
        request->question = question;
        request->location = location;

        // Wait for the service to become available
        while (!client_->wait_for_service(std::chrono::seconds(1))) {
            if (!rclcpp::ok()) {
                std::cout << "Interrupted while waiting for the service. Exiting." << std::endl;
                return "";  // Return empty string if service is not available
            }
            std::cout << "Waiting for service to be available..." << std::endl;
        }

        // Send the request asynchronously
        auto result_future = client_->async_send_request(request);

        // Non-blocking check if the future is ready
        if (result_future.wait_for(std::chrono::seconds(50)) == std::future_status::ready) {
            auto response =  result_future.get()->response;
            std::cout << "Response: '" << response << "'" << std::endl;
            return response;
        } else {
            std::cout << "Service not ready or timed out." << std::endl;
            return "";  // Return empty string if the result is not ready or timed out
        }
    }

    bool check_robot_at_loc(const std::string &loc) {
//        std::cout << "robot at: "<< robot_at << std::endl;
//        std::cout << "loc "<< loc << std::endl;
        return (loc == robot_at);
    }

    bool check_person_at_loc(const std::string &loc) {
//        std::cout << "person at: "<< person_at << std::endl;
//        std::cout << "loc "<< loc << std::endl;
        return (loc == person_at);
    }

    void terminate_node() {
        std::lock_guard<std::mutex> lock(terminate_mtx);
        terminate_node_ = true;
    }

    bool should_terminate_node() {
        std::lock_guard<std::mutex> lock(terminate_mtx);
        return terminate_node_;
    }

    shr_parameters::Params get_params() {
        std::lock_guard<std::mutex> lock(world_state_mtx);
        return param_listener_->get_params();
    }

    std::shared_ptr<shr_msgs::msg::WorldState> get_world_state_msg() {
        std::lock_guard<std::mutex> lock(world_state_mtx);
        return world_state_;
    }
};
