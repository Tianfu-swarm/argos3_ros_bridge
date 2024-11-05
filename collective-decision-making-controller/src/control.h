/*
 * control.h
 *
 *  Created on: 16 Jul 2024
 *  Author: Sindiso Mkhatshwa
 * 	Email: sindiso.mkhatshwa@uni-konstanz.de
 */

#ifndef CONTROL_H_
#define CONTROL_H_

#include <string>
#include <chrono>
#include <memory>
#include <iostream>
#include <sstream>
#include <random>
#include <climits>
#include <algorithm>

/* Vector2 definitions */
#include <argos3/core/utility/math/vector2.h>
#include <argos3/core/utility/math/vector3.h>
#include <argos3/core/utility/math/quaternion.h>
#include <argos3/core/utility/datatypes/byte_array.h>

#include "util.h"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/parameter.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "collective_decision_making/msg/led.hpp"
#include "collective_decision_making/msg/signal.hpp"
#include "collective_decision_making/msg/light.hpp"
#include "collective_decision_making/msg/light_list.hpp"
#include "collective_decision_making/msg/blob.hpp"
#include "collective_decision_making/msg/blob_list.hpp"
#include "collective_decision_making/msg/position.hpp"
#include "collective_decision_making/msg/proximity.hpp"
#include "collective_decision_making/msg/proximity_list.hpp"
#include "collective_decision_making/msg/packet.hpp"
#include "collective_decision_making/msg/packet_list.hpp"
#include "ament_index_cpp/get_package_share_directory.hpp"

/* Function definitions for XML parsing */
#include <argos3/core/utility/configuration/argos_configuration.h>
#include <argos3/core/utility/logging/argos_log.h>

using namespace argos;
using namespace collective_decision_making::msg;
using namespace geometry_msgs::msg;

class Control{
public:
	struct SWheelTurningParams {
	  /*
	   * The turning mechanism.
	   * The robot can be in three different turning states.
	   */
	  enum ETurningMechanism
	  {
		 NO_TURN = 0, // go straight
		 SOFT_TURN,   // both wheels are turning forwards, but at different speeds
		 HARD_TURN    // wheels are turning with opposite speeds
	  } TurningMechanism;
	  /*
	   * Angular thresholds to change turning state.
	   */
	  CRadians HardTurnOnAngleThreshold;
	  CRadians SoftTurnOnAngleThreshold;
	  CRadians NoTurnAngleThreshold;
	  /* Maximum wheel speed */
	  float MaxSpeed;

	  void Init(/**TConfigurationNode& t_tree*/);
   };
	enum robotState
	{
	 TO_TARGET = 0, // go straight
	 AVOID,   // steer clear of obstacle
	 //WANDER
	};

	/* Enum for received message type*/
	typedef enum {
		UNCOMMITTED_MSG=0,
		RECRUITMENT_MSG,
		INHIBITION_MSG,
	} rxMessageType;

	/* Enum for type of inhibition used in this experiment */
	typedef enum {
		PASSIVE = 0,
		BUFFER,
	} communicationType;

	/** Enum for the communication type used in this experiment */
	typedef enum {
		CROSSINHIBITION = 0,
		DIRECTSWITCH
	} inhibitionType;
	/* Coordinate point */
	struct Point{
		float x;
		float y;
		Point(float x, float y);
		Point();
	};

	/* Coordinate point */
	struct Target{
		Point coords;
		int id;
		Target(float x, float y, int id);
		Target(Point coords, int id);
		Target();
	};
	/**
	 * Constants
	 */
	const static int STATE_TIME_OUT = 10;//50;
	const static int MAX_FORWARD_SPEED = 1;
	static constexpr float MAX_ROTATION_SPEED = 2.5f;

	/** Class constructor */
	Control(std::shared_ptr<rclcpp::Node> node);

	/* Class destructor. */
	virtual ~Control() {}

	void lightCallback(const LightList lightList);

	void proxCallback(const ProximityList proxList);

	void posCallback(const Position position);

	void rabCallback(const PacketList packets);

	void blobCallback(const BlobList blobList);

	void killCallback(const Signal sig);

	void transition(robotState  newState);

	Twist twistRandom();

	Twist twistTowardsThing(float angle, bool backwards);

	Twist SetWheelSpeedsFromVector(const CVector2 &c_heading);

	void initTargets();

	Packet broadcast(bool uncommitted = false);

	void setCommitmentOpinions();

	void setCommitmentPerception();

	void updateCommitment();

	void initializeParameters();

	void configure();

	void initLogging();

	void log();

private:
	/**************************************
	 * Create topic subscribers
	 *************************************/
	// Light list subscriber
	rclcpp::Subscription<LightList>::SharedPtr lightListSubscriber_;
	// colored-blob-omnidirectional-camera sensor subscriber
	rclcpp::Subscription<ProximityList>::SharedPtr proxSubscriber_;
	// positioning sensor subscriber
	rclcpp::Subscription<Position>::SharedPtr posSubscriber_;
	// rab sensor subscriber
	rclcpp::Subscription<PacketList>::SharedPtr rabSubscriber_;
	// colored-blob-omnidirectional-camera sensor subscriber
	rclcpp::Subscription<BlobList>::SharedPtr blobSubscriber_;
	// kill signal subscriber
	rclcpp::Subscription<Signal>::SharedPtr killSubscriber_;
	/**************************************
	 * Create topic publishers
	 **************************************/
	rclcpp::Publisher<Twist>::SharedPtr cmdVelPublisher_;
	// We use a Packet because we cannot just publish a CByteArray (ROS does not support it)
	rclcpp::Publisher<Packet>::SharedPtr cmdRabPublisher_;
	// We use a Led interface to publish the desired LED color
	rclcpp::Publisher<Led>::SharedPtr cmdLedPublisher_;

	std::shared_ptr<rclcpp::Node> node_;

	const char* ns_;

	robotState state_;

	inhibitionType inhibitionType_;
	communicationType commsType_;


	int time_;
	int stateStartTime_;

	Twist lastTwist_;

	LightList lightList;
	ProximityList proximityList;
	BlobList blobList;

	PacketList rxPacketList_;

	/* Current position of robot */
	Position curr_position;

	std::random_device dev;

	/* The turning parameters. */
	SWheelTurningParams m_sWheelTurningParams;

	/*****************************************
	* Communication variables
	*****************************************/
	/* current commitment */
	uint8_t targetCommitment_;

	/* Location coordinates of current commitment */
	Point targetGPS_;

	Target commitment_;

	/* Location coordinates of received commitment */
	Point rxTargetGPS_;
	Target rxCommitment_;

	bool rxMessage_;
	rxMessageType rxMsgType_;

	/* The two target locations */
	Point targets_[2];

	/* The two target locations */
	std::string lightSources_[2];

	uint32_t broadcastTime_;

	uint32_t commitmentUpdateTime_;

	// Robot id - commitment
	std::map<int, int> msgBuffer_;

	// Probability to update commitment using perception
	float pPerceiveLightSources_;

	std::string gStartTime_;

	// For logging purposes
	std::vector<int> opinionsList;
	//std::ofstream gLogFile;

	//std::string gLogDirectoryname;
	//std::string gLogFilename;
	//std::string gLogFullFilename; // cf. the initL


};



#endif /* CONTROL_H_ */
