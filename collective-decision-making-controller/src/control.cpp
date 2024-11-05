/*
 * control.cpp
 *
 *  Created on: 16 Jul 2024
 *      Author: rooot
 */
#include "control.h"

using namespace std;

using namespace collective_decision_making::msg;
using namespace geometry_msgs::msg;
using std::placeholders::_1;

std::ofstream gLogFile;

void Control::SWheelTurningParams::Init() {
	TurningMechanism = NO_TURN;
	HardTurnOnAngleThreshold = ToRadians(CDegrees(50));//25,50
	SoftTurnOnAngleThreshold = ToRadians(CDegrees(10));//22.5, 45
	NoTurnAngleThreshold = ToRadians(CDegrees(5));//15,30
	MaxSpeed = 10;
}

Control::Point::Point(float x, float y){this->x=x;this->y=y;};
Control::Point::Point(){this->x=0;this->y=0;};

Control::Target::Target(float x, float y, int id){this -> coords.x = x; this -> coords.y = y; this -> id = id;}
Control::Target::Target(Point coords, int id) : coords(coords), id(id){}
Control::Target::Target() : id(-1), coords(-1, -1){}

Control::Control(std::shared_ptr<rclcpp::Node> node) :
	time_(0),
	stateStartTime_(0),
	//broadcastTime_(10),
	//commitmentUpdateTime_(25),
	rxMessage_(false)
	{
	this -> node_ = node;
	// Declare parameters.
    this->initializeParameters();

    this->configure();
	commsType_ = communicationType::PASSIVE;
	//if ( commsType_ == communicationType::PASSIVE )
		//cout << "Passive - Comms" << std::endl;
	//else
		//cout << "Buffer - Comms" << std::endl;
	this -> commitment_.id = -1;

	this -> rxPacketList_.n = 0;

	
	gStartTime_ = getCurrentTimeAsReadableString();

	// Go to nearest light source
	state_ = TO_TARGET;
	ns_ = node_->get_namespace();

	initLogging();

	// Create the topic to publish
	stringstream cmdVelTopic, cmdRabTopic, cmdLedTopic;
	cmdVelTopic << ns_ << "/cmd_vel";
	cmdRabTopic << ns_ << "/cmd_rab";
	cmdLedTopic << ns_ << "/cmd_led";

	cmdVelPublisher_ = node_ -> create_publisher<Twist>(cmdVelTopic.str(), 1);
	cmdRabPublisher_ = node_ -> create_publisher<Packet>(cmdRabTopic.str(), 1);
	cmdLedPublisher_ = node_ -> create_publisher<Led>(cmdLedTopic.str(), 1);
	// Create the subscribers
	stringstream lightListTopic, proxTopic, posTopic, rabTopic, blobTopic, killTopic;
	lightListTopic << ns_ << "/light";
	proxTopic << ns_ << "/proximity";
	posTopic << ns_ << "/position";
	rabTopic << ns_ << "/rab";
	blobTopic << ns_ << "/blob";
	killTopic << "/kill";

	lightListSubscriber_ = node_ -> create_subscription<collective_decision_making::msg::LightList>(
				lightListTopic.str(),
				1,
				std::bind(&Control::lightCallback, this, _1)
				);

	proxSubscriber_	= node_ -> create_subscription<collective_decision_making::msg::ProximityList>(
			proxTopic.str(),
			1,
			std::bind(&Control::proxCallback, this, _1)
			);
	posSubscriber_ 	= node_ -> create_subscription<collective_decision_making::msg::Position>(
			posTopic.str(),
			1,
			std::bind(&Control::posCallback, this, _1)
			);
	rabSubscriber_ 	= node_ -> create_subscription<PacketList>(
			rabTopic.str(),
			1,
			std::bind(&Control::rabCallback, this, _1)
			);
	blobSubscriber_	= node_ -> create_subscription<BlobList>(
			blobTopic.str(),
			1,
			std::bind(&Control::blobCallback, this, _1)
			);
	killSubscriber_	= node_ -> create_subscription<Signal>(
			killTopic.str(),
			1,
			std::bind(&Control::killCallback, this, _1)
			);
	// This should never happen!!!
 	for ( Packet currPacket : rxPacketList_.packets ){
		std::cout << "At init - id: " << currPacket.data[1] << std::endl;
	}

}

void Control::transition(robotState newState){
	this -> state_ = newState;
	this ->  stateStartTime_ = this -> time_;
}

void Control::initTargets(){
	lightSources_[0] = "yellow";
	lightSources_[1] = "green";
	this -> commitment_.id = std::stoi( std::string(ns_).substr (4) ) <= 10 ? 1 : 2;
	this -> targetCommitment_ = 0;
	
	Led color;
	color.color= this -> commitment_.id == 1 ? "yellow" : "green";
	this -> cmdLedPublisher_ -> publish(color);

}

void Control::initLogging(){
	// ==== create specific "maps" logger file
	Logger:: gLogFilename = Logger:: gExperiementname + "_" + std::string(ns_).substr (1) + "_" + gStartTime_ + "_" + getpidAsReadableString() + ".csv";
	Logger::gLogFullFilename = Logger::gLogDirectoryname + "/" + Logger::gLogFilename;
	Logger::gRobotStateLogFile.open(Logger::gLogFullFilename.c_str());

	if(!Logger::gRobotStateLogFile) {
		std::cout << "[CRITICAL] Cannot open \"robot state\" log file " << Logger::gLogFullFilename << "." << std::endl;
		exit(-1);
	}
	std::cout << "Log filename: " << Logger::gLogFullFilename << std::endl;
	Logger::gRobotStateLogger = new Logger();
	Logger::gRobotStateLogger->setLoggerFile(Logger::gRobotStateLogFile);
	Logger::gRobotStateLogger->write("Time, Commitment, Opinion, Light-Source-in-Sight");
	Logger::gRobotStateLogger->write(std::string("\n"));
	Logger::gRobotStateLogger->flush();
}

Twist Control::twistTowardsThing(float angle, bool backwards=false){
	float v = 0.0f;
	float w = 0.0f;

	if ( std::abs(angle) < 0.5 ){
		// The object is roughly in-front, go "towards" it
		if (backwards){
			v = -MAX_FORWARD_SPEED;
		}
		else{
			v = MAX_FORWARD_SPEED;
		}
	}
	else if ( angle < 0){
		// Turn right
		w = - MAX_ROTATION_SPEED;
	}
	else if (angle > 0){
		// Turn left
		w = MAX_ROTATION_SPEED;
	}
	Twist twist;
	twist.linear.x = v;
	twist.linear.z = 1.0; // AVOID state-flag
	twist.angular.z = w;
	return twist;
}

Twist Control::SetWheelSpeedsFromVector(const CVector2& c_heading) {
   /* Get the heading angle */
   CRadians cHeadingAngle = c_heading.Angle().SignedNormalize();
   /* Get the length of the heading vector */
   Real fHeadingLength = c_heading.Length();
   /* Clamp the speed so that it's not greater than MaxSpeed */
   Real fBaseAngularWheelSpeed = Min<Real>(fHeadingLength, m_sWheelTurningParams.MaxSpeed);

   /* State transition logic */
   if(m_sWheelTurningParams.TurningMechanism == SWheelTurningParams::HARD_TURN) {
      if(Abs(cHeadingAngle) <= m_sWheelTurningParams.SoftTurnOnAngleThreshold) {
         m_sWheelTurningParams.TurningMechanism = SWheelTurningParams::SOFT_TURN;
      }
   }
   if(m_sWheelTurningParams.TurningMechanism == SWheelTurningParams::SOFT_TURN) {
      if(Abs(cHeadingAngle) > m_sWheelTurningParams.HardTurnOnAngleThreshold) {
         m_sWheelTurningParams.TurningMechanism = SWheelTurningParams::HARD_TURN;
      }
      else if(Abs(cHeadingAngle) <= m_sWheelTurningParams.NoTurnAngleThreshold) {
         m_sWheelTurningParams.TurningMechanism = SWheelTurningParams::NO_TURN;
      }
   }
   if(m_sWheelTurningParams.TurningMechanism == SWheelTurningParams::NO_TURN) {
      if(Abs(cHeadingAngle) > m_sWheelTurningParams.HardTurnOnAngleThreshold) {
         m_sWheelTurningParams.TurningMechanism = SWheelTurningParams::HARD_TURN;
      }
      else if(Abs(cHeadingAngle) > m_sWheelTurningParams.NoTurnAngleThreshold) {
         m_sWheelTurningParams.TurningMechanism = SWheelTurningParams::SOFT_TURN;
      }
   }
   /* Wheel speeds based on current turning state */
   Real fSpeed1, fSpeed2;
   switch(m_sWheelTurningParams.TurningMechanism) {
      case SWheelTurningParams::NO_TURN: {
         /* Just go straight */
         fSpeed1 = fBaseAngularWheelSpeed;
         fSpeed2 = fBaseAngularWheelSpeed;
         /**
		  * Broadcast  opinion and update opinion state
		  */
		if ( this -> time_ > 0 && this -> time_ % this -> broadcastTime_ == 0 && this -> commitment_.id != -1 ){
			this -> cmdRabPublisher_ -> publish(broadcast());
		}
         break;
      }
      case SWheelTurningParams::SOFT_TURN: {
         /* Both wheels go straight, but one is faster than the other */
         Real fSpeedFactor = (m_sWheelTurningParams.HardTurnOnAngleThreshold - Abs(cHeadingAngle)) / m_sWheelTurningParams.HardTurnOnAngleThreshold;
         fSpeed1 = fBaseAngularWheelSpeed - fBaseAngularWheelSpeed * (1.0 - fSpeedFactor);
         fSpeed2 = fBaseAngularWheelSpeed + fBaseAngularWheelSpeed * (1.0 - fSpeedFactor);
         /**
		  * Broadcast  opinion and update opinion state
		  */
		if ( this -> time_ > 0 && this -> time_ % this -> broadcastTime_ == 0 && this -> commitment_.id != -1 ){
			this -> cmdRabPublisher_ -> publish(broadcast());
		}
         break;
      }
      case SWheelTurningParams::HARD_TURN: {
    	  /** We can broadcast while doing a hard turn but only when utilizing PASSIVE communication */
    	  if ( this -> time_ > 0 && this -> time_ % this -> broadcastTime_ == 0
    			  && this -> commitment_.id != -1 && this -> commsType_ == PASSIVE ){
			this -> cmdRabPublisher_ -> publish(broadcast(true));
    	  }
         /* Opposite wheel speeds */
         fSpeed1 = -m_sWheelTurningParams.MaxSpeed;
         fSpeed2 =  m_sWheelTurningParams.MaxSpeed;
         break;
      }
   }
   /* Apply the calculated speeds to the appropriate wheels */
   Real fLeftWheelSpeed, fRightWheelSpeed;
   if(cHeadingAngle > CRadians::ZERO) {
      /* Turn Left */
      fLeftWheelSpeed  = fSpeed1;
      fRightWheelSpeed = fSpeed2;
   }
   else {
      /* Turn Right */
      fLeftWheelSpeed  = fSpeed2;
      fRightWheelSpeed = fSpeed1;
   }
   /* Finally, set the wheel speeds */
   Twist twist;
   twist.linear.x = fLeftWheelSpeed;
   twist.linear.y = fRightWheelSpeed;
   return twist;
}

Packet Control::broadcast(bool uncommitted){
	Packet packet;
	
	float targetToBroadcast = uncommitted? 0.0f : float( this -> commitment_.id );
	// For logging purposes
	if (uncommitted){
		opinionsList.push_back(0);
	}
	else{
		opinionsList.push_back(commitment_.id);
		
	}
	
	packet.data.push_back(targetToBroadcast);
	packet.id = std::string(ns_).substr (4);

	return packet;

}

void Control::setCommitmentOpinions() {

	std::mt19937 rng(this -> dev());
	std::uniform_int_distribution<std::mt19937::result_type> dist6(0, this -> rxPacketList_.n); // distribution in range [1, 100]
	int rand = dist6(rng);

	size_t index = 0;
	for ( Packet currPacket : rxPacketList_.packets ){

		if ( index == rand ){

			this -> rxCommitment_ = Target(
					float(0),
					float(0),
					int(currPacket.data[0]));
			if ( rxCommitment_.id != 0 ){
				rxMsgType_ = RECRUITMENT_MSG;
			}
			else {
				rxMsgType_ = UNCOMMITTED_MSG;
			}
			this -> rxMessage_ = true;

		}
		index++;
	}
		if ( rxMessage_ && this -> commitment_.id != this -> rxCommitment_.id  && rxMsgType_ == RECRUITMENT_MSG) {			
			this -> commitment_ = Target(rxCommitment_.coords, rxCommitment_.id);
			//cout << "Time: " << time_ << " Opinion new commitment id: " << rxCommitment_.id << std::endl;
			Led color;
			color.color= this -> commitment_.id == 1 ? "yellow" : "green";
			this -> cmdLedPublisher_ -> publish(color);
		}

}

void Control::setCommitmentPerception(){
	int blobsInSightCount = 0;
	for ( Blob blob : blobList.blobs ){
		if ( blob.color == "yellow" || blob.color == "green" ){
			blobsInSightCount++;
		}
	}
	if (blobsInSightCount > 0){
		std::mt19937 rng(this -> dev());
		std::uniform_int_distribution<std::mt19937::result_type> dist6(1, blobsInSightCount); // distribution in range [1, 100]
		int rand = dist6(rng);

		this -> commitment_ = Target(0, 0, rand);
		//cout << "Time: " << time_ <<" Perception new commitment id: " << commitment_.id << std::endl;
		Led color;
		color.color= this -> commitment_.id == 1 ? "yellow" : "green";
		this -> cmdLedPublisher_ -> publish(color);

	}


}

void Control::updateCommitment() {

	float dice = float(randint()%100) / 100.0;
	if ( dice < pPerceiveLightSources_ ){
		setCommitmentPerception();
	}
	else{
		setCommitmentOpinions();
	}

	rxMessage_ = false;
	this -> rxPacketList_.packets.clear();
	this -> rxPacketList_.n = 0;
	for ( Packet currPacket : rxPacketList_.packets ){
		std::cout << "Commitment ID not deleted: " << this -> rxCommitment_.id << std::endl;
	}
	msgBuffer_.clear();
	opinionsList.clear();

}

void Control::initializeParameters(){
	/**
	 * This parameter sets the duration of a broadcast
	 * Default: 10 ticks
	 */
	rcl_interfaces::msg::ParameterDescriptor broadcastTimeDescriptor;
    broadcastTimeDescriptor.name = "broadcastTime";
    broadcastTimeDescriptor.type = rcl_interfaces::msg::ParameterType::PARAMETER_INTEGER;
    broadcastTimeDescriptor.description = "Number of ticks before an agent can broadcast an opinion.";
    this -> node_ -> declare_parameter("broadcastTime", 10, broadcastTimeDescriptor);

	/**
	 * This parameter sets the duration of an agent's commitment to a target
	 * Default: 25 ticks
	 */
	rcl_interfaces::msg::ParameterDescriptor commitmentUpdateTimeDescriptor;
    commitmentUpdateTimeDescriptor.name = "commitmentUpdateTime";
    commitmentUpdateTimeDescriptor.type = rcl_interfaces::msg::ParameterType::PARAMETER_INTEGER;
    commitmentUpdateTimeDescriptor.description = "Number of ticks before an agent can update its commitment.";
    this -> node_ -> declare_parameter("commitmentUpdateTime", 25, commitmentUpdateTimeDescriptor);

	/**
	 * This parameter sets the probability to use the agents' perception
	 * to update commitments, i.e., instead of using received opions.
	 * Default: 0.1 (10%)
	 */
    rcl_interfaces::msg::ParameterDescriptor pPerceiveLightSourcesDescriptor;
    pPerceiveLightSourcesDescriptor.name = "pPerceiveLightSources";
    pPerceiveLightSourcesDescriptor.type = rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE;
    pPerceiveLightSourcesDescriptor.description = "Probability to update robot commitment based on agents' perception of light sources.";
    this -> node_ -> declare_parameter("pPerceiveLightSources", 0.1, pPerceiveLightSourcesDescriptor);

	/**
	 * This parameter sets the communication type between agents - Passive or Buffer.
	 * Default: Passive (Value of 0)
	 */
    rcl_interfaces::msg::ParameterDescriptor commsTypeDescriptor;
    commsTypeDescriptor.name = "commsType";
    commsTypeDescriptor.type = rcl_interfaces::msg::ParameterType::PARAMETER_INTEGER;
    commsTypeDescriptor.description = "Communication type: Passive - 0, Buffer - 1.";
    this -> node_ -> declare_parameter("commsType", 0, commsTypeDescriptor);

	/**
	 * This parameter sets the threshold to perform a hard turn.
	 * Default: 50 degrees.
	 */
    rcl_interfaces::msg::ParameterDescriptor hardTurnOnAngleThresholdDescriptor;
    hardTurnOnAngleThresholdDescriptor.name = "hardTurnOnAngleThreshold";
    hardTurnOnAngleThresholdDescriptor.type = rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE;
    hardTurnOnAngleThresholdDescriptor.description = "Threshold angle (degrees) to perform a hard turn.";
    this -> node_ -> declare_parameter("hardTurnOnAngleThreshold", 50.0, hardTurnOnAngleThresholdDescriptor);

	/**
	 * This parameter sets the threshold to perform a soft turn.
	 * Default: 50 degrees.
	 */
    rcl_interfaces::msg::ParameterDescriptor softTurnOnAngleThresholdDescriptor;
    softTurnOnAngleThresholdDescriptor.name = "softTurnOnAngleThreshold";
    softTurnOnAngleThresholdDescriptor.type = rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE;
    softTurnOnAngleThresholdDescriptor.description = "Threshold angle (degrees) to perform a soft turn.";
    this -> node_ -> declare_parameter("softTurnOnAngleThreshold", 10.0, softTurnOnAngleThresholdDescriptor);

	/**
	 * This parameter sets the no-turn threshold.
	 * Default: 5 degrees.
	 */
    rcl_interfaces::msg::ParameterDescriptor noTurnOnAngleThresholdDescriptor;
    noTurnOnAngleThresholdDescriptor.name = "noTurnOnAngleThreshold";
    noTurnOnAngleThresholdDescriptor.type = rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE;
    noTurnOnAngleThresholdDescriptor.description = "Threshold angle (degrees) for a no-turn.";
    this -> node_ -> declare_parameter("noTurnOnAngleThreshold", 5.0, noTurnOnAngleThresholdDescriptor);

	/**
	 * This parameter sets the maximum speed of robots.
	 * Default: 10.
	 */
    rcl_interfaces::msg::ParameterDescriptor maxSpeedDescriptor;
    maxSpeedDescriptor.name = "maxSpeed";
    maxSpeedDescriptor.type = rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE;
    maxSpeedDescriptor.description = "Maximum speed for robots.";
    this -> node_ -> declare_parameter("maxSpeed", 10.0, maxSpeedDescriptor);

	/****************************************
	 * Logging related parameters
	 ***************************************/
	/**
	 * This parameter sets the directory to save logs
	 * Default: 10 ticks
	 */
	rcl_interfaces::msg::ParameterDescriptor logDirectoryDescriptor;
    logDirectoryDescriptor.name = "logDirectoryName";
    logDirectoryDescriptor.type = rcl_interfaces::msg::ParameterType::PARAMETER_INTEGER;
    logDirectoryDescriptor.description = "Directory to save logs during experiment run.";
	const std::string package_name = "controller";
	std::string defaultDirectory = ament_index_cpp::get_package_share_directory(package_name);
    this -> node_ -> declare_parameter("logDirectoryName", defaultDirectory, logDirectoryDescriptor);

	/**
	 * This parameter sets the experiment name
	 * Default: current time 
	 */
	rcl_interfaces::msg::ParameterDescriptor experimentNameDescriptor;
    experimentNameDescriptor.name = "experimentName";
    experimentNameDescriptor.type = rcl_interfaces::msg::ParameterType::PARAMETER_INTEGER;
    experimentNameDescriptor.description = "Experiment run name.";
	std::string defaultExperimentName = std::string(gStartTime_);
    this -> node_ -> declare_parameter("experimentName", defaultExperimentName, experimentNameDescriptor);
	
}

void Control::configure(){
	rclcpp::Logger node_logger = this -> node_ -> get_logger();

	m_sWheelTurningParams.TurningMechanism = SWheelTurningParams::ETurningMechanism::NO_TURN;

    this -> node_ -> get_parameter<std::uint32_t>("broadcastTime", broadcastTime_);
    //RCLCPP_INFO(node_logger, "broadcast time: %d", broadcastTime_);

	this -> node_ -> get_parameter<std::uint32_t>("commitmentUpdateTime", commitmentUpdateTime_);
    //RCLCPP_INFO(node_logger, "commitment update time: %d", commitmentUpdateTime_);

	this -> node_ -> get_parameter<float>("pPerceiveLightSources", pPerceiveLightSources_);
    //RCLCPP_INFO(node_logger, "probability to perceive light sources for update: %f", pPerceiveLightSources_);

	int comms;
	this -> node_ -> get_parameter<int>("commsType", comms);
    //RCLCPP_INFO(node_logger, "communication type: %d", comms);
	commsType_ = (comms==0)? communicationType::PASSIVE : communicationType::BUFFER;

	float angle;
	this -> node_ -> get_parameter<float>("hardTurnOnAngleThreshold", angle);
	//RCLCPP_INFO(node_logger, "hard-turn on angle threshold: %f", angle);
	m_sWheelTurningParams.HardTurnOnAngleThreshold = ToRadians(CDegrees(angle));
    
	this -> node_ -> get_parameter<float>("softTurnOnAngleThreshold", angle);
	//RCLCPP_INFO(node_logger, "soft-turn on angle threshold: %f", angle);
	m_sWheelTurningParams.SoftTurnOnAngleThreshold = ToRadians(CDegrees(angle));

	this -> node_ -> get_parameter<float>("noTurnOnAngleThreshold", angle);
	//RCLCPP_INFO(node_logger, "no-turn on angle threshold: %f", angle);
	m_sWheelTurningParams.NoTurnAngleThreshold = ToRadians(CDegrees(angle));

    this -> node_ -> get_parameter<float>("maxSpeed", m_sWheelTurningParams.MaxSpeed);
    //RCLCPP_INFO(node_logger, "maximum speed: %f", m_sWheelTurningParams.MaxSpeed);

	/************************************************
	 * Logs related configurations
	 **********************************************/
	this -> node_ -> get_parameter<string>("logDirectoryName", Logger::gLogDirectoryname);
	std::cout << "log directory: " << Logger::gLogDirectoryname << std::endl;

	this -> node_ -> get_parameter<string>("experimentName", Logger::gExperiementname);
	std::cout << "experiment name: " << Logger::gExperiementname << std::endl;
}

/**************************
 * Light callback function
 *************************/
void Control::lightCallback(const LightList lightList){
	this->lightList = lightList;
}

/**************************
 * Position callback function
 *************************/
void Control::posCallback(const Position position){
	this -> curr_position = position;
}

/**************************
 * Blob callback function
 *************************/
void Control::blobCallback(const BlobList blobList){
	this -> blobList = blobList;
}

/**************************
 * Kill signal callback function
 *************************/
void Control::killCallback(const Signal sig){
	if (sig.signal == 1){
		std::cout << "received kill signal" << std::endl;
		rclcpp::shutdown();
		exit(0);
	}
}

/**************************
 * Range and Bearing callback function
 *************************/
void Control::rabCallback(const PacketList packets){
	/* This check is important to ensure that we do not pick up
	 * empty RAB sensor values - consider delay between ros2 and argos
	 */
	if ( time_ > this->broadcastTime_ +4 ){
		for ( Packet currPacket : packets.packets ){
			/**
			 * If it is a new agent, insert new record
			 */
			if (msgBuffer_.insert({int(currPacket.data[1]), int(currPacket.data[0])}).second){
				this -> rxPacketList_.packets.push_back(currPacket);
				this -> rxPacketList_.n ++;
			}
			/**
			 * If it is an existing record, update the agent commitment.
			 */
			else{
				msgBuffer_[int(currPacket.data[1])] = int(currPacket.data[0]);
			}
	}
	}

}

/**************************
 * Proximity callback function
 *************************/
void Control::proxCallback(const ProximityList proxList){
	if (time_ == 0 || this -> commitment_.id == -1){
		initTargets();
		//cout << "Time: " << time_ << " Initializing targets" << std::endl;
	}

	this -> time_ ++;

	//uint32_t ros::Publisher::getNumSubscribers() const

	/**
	 * Find the closest obstacle (other robot or wall).  The closest obstacle
	 * is the one with the greatest 'value'.
	 */
	Proximity closestObs;
	bool closestObsIsNull = true;
	float highestValue = 0.0f;
	for ( Proximity currProximity : proxList.proximities ){
		if (currProximity.value > highestValue){
			closestObsIsNull = false;
			closestObs = currProximity;
			highestValue = currProximity.value;
		}
	}

	Light closestLight;
	bool closestLightIsNull = true;
	highestValue = 0.0f;
	if ( this -> lightList.n > 0 ){ // just in case the light is not picked up
		for ( Light currLight : lightList.lights ){
			if ( currLight.value > highestValue){
				closestLightIsNull = false;
				closestLight = currLight;
				highestValue = currLight.value;
			}
		}
	}

	Blob closestBlob;
	bool closestBlobIsNull = true;
	float closestDist = std::numeric_limits<float>::infinity();
	if ( this -> blobList.n > 0 ){ // just in case the light is not picked up
			/**
			 * Not committed to a light source yet
			 */
			if ( this -> commitment_.id == -1){
				for ( Blob blob : blobList.blobs ){
					if ( blob.distance < closestDist && blob.color != "red"){
						closestBlobIsNull = false;
						closestBlob = blob;
						closestDist = blob.distance;
					}
				}
				commitment_.id = closestBlob.color == "yellow" ? 1 : 2;
				targetCommitment_ = closestBlob.color == "yellow" ? 1 : 2;
				Led color;
				color.color= this -> commitment_.id == 1 ? "yellow" : "green";
				
				this -> cmdLedPublisher_ -> publish(color);

			}
			/**
			 * Already committed to a light source. Just find new vector to light source
			 */
			else{
				for ( Blob blob : blobList.blobs ){
					if ( blob.color == lightSources_[commitment_.id - 1] ){
						closestBlobIsNull = false;
						closestBlob = blob;
					}
				}
			}
	}

	/**
	 * Handle the state transitions
	 */
	if (this ->  state_ == robotState::AVOID){
		// Only leave upon time out
		if ( this -> time_ - this -> stateStartTime_ > STATE_TIME_OUT && ! closestBlobIsNull ){
			transition(robotState::TO_TARGET);
		}
	}
	else if (this ->  state_ == robotState::TO_TARGET){
		if ( ! closestObsIsNull){
			transition(robotState::AVOID);
		}
	}
	else{
		std::cerr << "Error: Invalid state" << std::endl;
	}

	/**
	 * Handle state actions
	 */

	Twist twist;

	if ( this -> state_ == robotState::AVOID ){
		if (closestObsIsNull){
			twist = this -> lastTwist_;
		}
		else{
			twist = twistTowardsThing(closestObs.angle, true);
		}
		/**
		 * If it is time to broadcast just broadcast nonentheless
		 */
		if ( this -> time_ > 0 && this -> time_ % this -> broadcastTime_ == 0
				&& this -> commitment_.id != -1 && this -> commsType_ == PASSIVE ){
			this -> cmdRabPublisher_ -> publish(broadcast(true));
		}
	}

	else if ( this -> state_ == robotState::TO_TARGET ){
		CVector2 t(closestBlob.distance/100.0f, CRadians(closestBlob.angle));
		twist = SetWheelSpeedsFromVector(CVector2(t.GetX(), t.GetY()));
	}
	else{
		std::cerr << "Error: Invalid state" << std::endl;
	}
	this -> cmdVelPublisher_ -> publish (twist);
	this -> lastTwist_ = twist;
	/**
	 * Update commitment
	 */
	if ( this -> commsType_ == PASSIVE){
		if ( this -> time_ > 0 && this -> time_ % this -> commitmentUpdateTime_ == 0 ) {
			log();
			updateCommitment();
		}
	}
	else {
		if (msgBuffer_.size() >= 3){
			log();
			this -> updateCommitment();
		}
		
	}



}

void Control::log(){
	int lightInSight = (this -> lightList.n > 0)? 1 : 0;

	/***
	 * Convert the opnions vector to a string
	 */
	std::ostringstream oss;
	if (!opinionsList.empty()){
		// Convert all but the last element to avoid a trailing ","
		std::copy(opinionsList.begin(), opinionsList.end()-1,
			std::ostream_iterator<int>(oss, "; "));

		// Now add the last element with no delimiter
		oss << opinionsList.back();
	}
	else{
		oss << "-";
	}
	
	stringstream logLine;
	logLine << time_ << ", " << commitment_.id << ", " << oss.str() << ", "<< lightInSight;
	Logger::gRobotStateLogger->write(logLine.str());
	Logger::gRobotStateLogger->write(std::string("\n"));
	Logger::gRobotStateLogger->flush();
}

/**************************
 * Main function
 *************************/
int main(int argc, char **argv) {
	rclcpp::init(argc, argv);
	std::shared_ptr<rclcpp::Node> node = std::make_shared<rclcpp::Node>("argos_ros_node");
	Control controller(node);
	rclcpp::spin(node);
	rclcpp::shutdown();
	return 0;

}
