/*
 * controller.cpp
 *
 *  Created on: 08 Jul 2024
 *  Author: Sindiso Mkhatshwa
 *  Email: sindiso.mkhatshwa@uni-konstanz.de
 */
#include <string>
#include <chrono>
#include <memory>
#include <iostream>
#include <sstream>
#include "rclcpp/rclcpp.hpp"
#include "collective_decision_making/msg/light.hpp"
#include "collective_decision_making/msg/proximity.hpp"
#include "collective_decision_making/msg/light_list.hpp"
#include "collective_decision_making/msg/proximity_list.hpp"

using namespace std;

using namespace collective_decision_making::msg;


class Controller{
public:
	Controller(std::shared_ptr<rclcpp::Node> node){

		// Create the topic to publish
		stringstream cmdVelTopic;
		cmdVelTopic << "/cmd_vel";
		cmdVelPublisher_ = node -> create_publisher<geometry_msgs::msg::Twist>(cmdVelTopic.str(), 1);

		// Create the subscribers
		stringstream lightListTopic, proximityTopic;
		lightListTopic << "/light_list";
		proximityTopic << "/proximity";

		lightListSubscriber_ = node -> create_subscription<collective_decision_making::msg::LightList>(
					lightListTopic.str(),
					1,
					std::bind(&Controller::lightCallback, this, _1)
					);

		promixitySubscriber_ = node -> create_subscription<collective_decision_making::msg::ProximityList>(
				proximityTopic.str(),
				1,
				std::bind(&Controller::proxCallback, this, _1)
				);
	}

	void lightCallback(const LightList proxList) {

	}
	void proxCallback(const ProximityList proxList) {

	}


private:
	rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmdVelPublisher_;
	// Light list subscriber
	rclcpp::Subscription<collective_decision_making::msg::LightList>::SharedPtr lightListSubscriber_;
	// Proximity sensor subscriber
	rclcpp::Subscription<collective_decision_making::msg::ProximityList>::SharedPtr promixitySubscriber_;

};

nt main(int argc, char **argv) {
	rclcpp::init(argc, argv);
	std::shared_ptr<rclcpp::Node> node = std::make_shared<rclcpp::Node>("argos_ros_node");
	Controller controller(node);
	rclcpp::spin(node);
	rclcpp::shutdown();
	return 0;

}
