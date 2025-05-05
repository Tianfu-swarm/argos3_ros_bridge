#ifndef ARGOS_ROS_FOOTBOT_H_
#define ARGOS_ROS_FOOTBOT_H_

/* Definition of the ARGoS Simulator singleton (controls simulation flow, time, etc.) */
#include <argos3/core/simulator/simulator.h>
/* Definition of the simulation space (manages entities, robot positions, etc.) */
#include <argos3/core/simulator/space/space.h>
/* Definition of the physics engine interface (manages physical simulation and time step size) */
#include <argos3/core/simulator/physics_engine/physics_engine.h>
/* Definition of the CCI_Controller class. */
#include <argos3/core/control_interface/ci_controller.h>
/* Definition of the differential steering actuator */
#include <argos3/plugins/robots/generic/control_interface/ci_differential_steering_actuator.h>
/* Definition of the omnidirectional camera sensor */
#include <argos3/plugins/robots/generic/control_interface/ci_colored_blob_omnidirectional_camera_sensor.h>
/* Definition of the range-and-bearing actuator */
#include <argos3/plugins/robots/generic/control_interface/ci_range_and_bearing_actuator.h>
/* Definition of the range-and-bearing sensor */
#include <argos3/plugins/robots/generic/control_interface/ci_range_and_bearing_sensor.h>
/* Definition of the foot-bot light sensor */
#include <argos3/plugins/robots/foot-bot/control_interface/ci_footbot_light_sensor.h>
/* Definition of the foot-bot proximity sensor */
#include <argos3/plugins/robots/foot-bot/control_interface/ci_footbot_proximity_sensor.h>
/* Definition of the positioning sensor */
#include <argos3/plugins/robots/generic/control_interface/ci_positioning_sensor.h>

/* Function definitions for XML parsing */
#include <argos3/core/utility/configuration/argos_configuration.h>

#include <string>
#include <chrono>
#include <memory>
#include <iostream>
#include <sstream>
#include <cuchar>

/**
 * ROS2 Imports
 */
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include <geometry_msgs/msg/pose_stamped.hpp>
#include "std_msgs/msg/float64_multi_array.hpp"
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include "std_msgs/msg/bool.hpp"
#include "rosgraph_msgs/msg/clock.hpp"
#include <tf2_ros/transform_broadcaster.h>

using namespace argos;
using namespace std::chrono_literals;

class ArgosRosFootbot : public CCI_Controller
{
private:
	// Proximity sensor publisher
	rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr promixityPublisher_;
	// Position sensor publisher
	rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr positionPublisher_;
	// rab data from boardcatsing
	rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr rabDataPublisher_;
	// sim clock publisher
	rclcpp::Publisher<rosgraph_msgs::msg::Clock>::SharedPtr clockPublisher_;
	//
	rclcpp::Publisher<tf2_msgs::msg::TFMessage>::SharedPtr tfPublisher_;

	/************************************
	 * Subscribers
	 ***********************************/
	// Subscriber for cmd_vel (Twist message) topic.
	rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmdVelSubscriber_;
	// Subscriber for cmd_rab (Float64MultiArray) topic
	rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr cmdRabSubscriber_;
	// Subscriber for Actuator message) topic
	rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr cmdActuatorSubscriber_;

	rclcpp::TimerBase::SharedPtr timer_;

	/* Pointer to the differential steering actuator */
	CCI_DifferentialSteeringActuator *m_pcWheels;
	/* Pointer to the foot-bot light sensor */
	CCI_FootBotLightSensor *m_pcLight;
	/* Pointer to proximity sensor */
	CCI_FootBotProximitySensor *m_pcProximity;
	/* Pointer to positioning sensor */
	CCI_PositioningSensor *m_pcPosition;
	/* Pointer to the range-and-bearing sensor */
	CCI_RangeAndBearingSensor *m_pcRABS;
	/* Pointer to the range-and-bearing actuator */
	CCI_RangeAndBearingActuator *m_pcRABA;
	/* Pointer to the pose message */
	geometry_msgs::msg::PoseStamped *m_pose;

	// The following constant values were copied from the argos source tree from
	// the file src/plugins/robots/foot-bot/simulator/footbot_entity.cpp
	static constexpr Real HALF_BASELINE = 0.07f; // Half the distance between wheels
	static constexpr Real WHEEL_RADIUS = 0.029112741f;

	/*
	 * The following variables are used as parameters for the
	 * algorithm. You can set their value in the <parameters> section
	 * of the XML configuration file, under the
	 * <controllers><argos_ros_bot_controller> section.
	 */

	// The number of time steps from the time step of the last callback
	// after which leftSpeed and rightSpeed will be set to zero.  Useful to
	// shutdown the robot after the controlling code on the ROS side has quit.
	int stopWithoutSubscriberCount;

	// The number of time steps since the last callback.
	int stepsSinceCallback;

	// Most recent left and right wheel speeds, converted from the ROS twist
	// message.
	Real leftSpeed, rightSpeed;
	std::vector<double> pendingRabData;

public:
	ArgosRosFootbot();

	virtual ~ArgosRosFootbot();

	/*
	 * This function initializes the controller.
	 * The 't_node' variable points to the <parameters> section in the XML
	 * file in the <controllers><footbot_ccw_wander_controller> section.
	 */
	virtual void Init(TConfigurationNode &t_node);
	/*
	 * This function is called once every time step.
	 * The length of the time step is set in the XML file.
	 */
	virtual void ControlStep();
	/*
	 * This function resets the controller to its state right after the
	 * Init().
	 * It is called when you press the reset button in the GUI.
	 * In this example controller there is no need for resetting anything,
	 * so the function could have been omitted. It's here just for
	 * completeness.
	 */
	virtual void Reset();
	/*
	 * Called to cleanup what done by Init() when the experiment finishes.
	 */
	virtual void Destroy() {}
	/*
	 * The callback method for getting new commanded speed on the cmd_vel topic.
	 */
	void cmdVelCallback(const geometry_msgs::msg::Twist &twist);
	/*
	 * The callback method for getting new commanded packet on the cmd_packet topic.
	 */
	void cmdRabCallback(const std_msgs::msg::Float64MultiArray &rabActuator);
	/*
	 * The callback method for getting new commanded led color on the cmd_led topic.
	 */

	/*
	 * The callback method for getting new commanded robot pose on the position topic.
	 */
	void cmdPoseCallback(const geometry_msgs::msg::PoseStamped &pose);

	static std::shared_ptr<rclcpp::Node> nodeHandle;
};

#endif /* ARGOS_ROS_FOOTBOT_H_ */
