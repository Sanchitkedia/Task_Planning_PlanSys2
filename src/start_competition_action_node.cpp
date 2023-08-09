// Copyright 2019 Intelligent Robotics Lab
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <memory>
#include <algorithm>

#include "plansys2_executor/ActionExecutorClient.hpp"

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include <ariac_msgs/msg/competition_state.hpp>
#include <std_srvs/srv/trigger.hpp>

using namespace std::chrono_literals;

class StartCompetitionAction : public plansys2::ActionExecutorClient
{
public:
    StartCompetitionAction()
        : plansys2::ActionExecutorClient("start_competition", 10ms)
    {
      competition_state_sub_ =
        this->create_subscription<ariac_msgs::msg::CompetitionState>(
            "/ariac/competition_state", 10,
            std::bind(&StartCompetitionAction::competition_state_cb, this,
                    std::placeholders::_1));
    }

    int competition_state_ = -1;  // Competition state
    bool competition_started_{false};   // Flag to check if competition is started

private:
    void competition_state_cb(const ariac_msgs::msg::CompetitionState::ConstSharedPtr msg) {
      competition_state_ = msg->competition_state;
    }
      
    void do_work()
    {
        if (competition_state_ == ariac_msgs::msg::CompetitionState::READY) {
            if (!competition_started_) {
                std::string srv_name = "/ariac/start_competition";

                std::shared_ptr<rclcpp::Node> node =
                    rclcpp::Node::make_shared("start_trigger_client");
                
                rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr client =
                    node->create_client<std_srvs::srv::Trigger>(srv_name);

                auto request = std::make_shared<std_srvs::srv::Trigger::Request>();

                while (!client->wait_for_service(std::chrono::milliseconds(1000))) {
                    if (!rclcpp::ok()) {
                    RCLCPP_ERROR(this->get_logger(),
                                    "Interrupted while waiting for the service. Exiting.");
                    }
                    RCLCPP_INFO_STREAM(this->get_logger(),
                                        "Service not available, waiting again...");
                }

                auto result = client->async_send_request(request);

                if (rclcpp::spin_until_future_complete(node, result) ==
                    rclcpp::FutureReturnCode::SUCCESS) {
                    RCLCPP_INFO_STREAM(this->get_logger(), "Starting Competition");
                    competition_started_ = true;
                    finish(true, 1.0, "Competition Started");
                } else {
                    RCLCPP_ERROR_STREAM(this->get_logger(), "Failed to call trigger service");
                }
            }
        }
    }

    rclcpp::Subscription<ariac_msgs::msg::CompetitionState>::SharedPtr
            competition_state_sub_;

};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<StartCompetitionAction>();

    node->set_parameter(rclcpp::Parameter("action_name", "startcompetition"));
    node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);

    rclcpp::spin(node->get_node_base_interface());

    rclcpp::shutdown();

    return 0;
}
