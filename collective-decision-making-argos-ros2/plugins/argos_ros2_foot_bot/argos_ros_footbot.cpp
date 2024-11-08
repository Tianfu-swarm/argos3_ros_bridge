/*
 * argos_ros_footbot.cpp
 *
 *  Created on: 20 Jun 2024
 *  Author: Sindiso Mkhatshwa
 *  Email: sindiso.mkhatshwa@uni-konstanz.de
 */

/* Include the controller definition */
#include "argos_ros_footbot.h"
using namespace std;
using namespace collective_decision_making;
using namespace collective_decision_making::msg;
using namespace geometry_msgs::msg;
using std::placeholders::_1;

/**
 * Initialize the node before creating the publishers
 * and subscribers. Otherwise we get a guard-error during
 * compilation if we initialize the node after.
 */
std::shared_ptr<rclcpp::Node> initNode() {
  int argc = 1;
  char *argv = (char *) "";
  if (rclcpp::get_contexts().empty()){rclcpp::init(argc, &argv);}
  
  return std::make_shared<rclcpp::Node>("argos_ros_node");

}

std::shared_ptr<rclcpp::Node> ArgosRosFootbot::nodeHandle = initNode();

ArgosRosFootbot::ArgosRosFootbot() :
		m_pcWheels(NULL),
		m_pcLight(NULL),
		m_pcLEDs(NULL),
		m_pcCamera(NULL),
		m_pcProximity(NULL),
		m_pcPosition(NULL),
		m_pcRABA(NULL),
		m_pcRABS(NULL),
		stopWithoutSubscriberCount(10),
		stepsSinceCallback(0),
		leftSpeed(0),
		rightSpeed(0){}

ArgosRosFootbot::~ArgosRosFootbot(){}

void ArgosRosFootbot::Init(TConfigurationNode& t_node){
	/********************************
	 * Create the topics to publish
	 *******************************/
	stringstream lightTopic, blobTopic, proxTopic, positionTopic, rabTopic, rabDataTopic;
	lightTopic 		<< "/" << GetId() << "/light";
	blobTopic 		<< "/" << GetId() << "/blob";
	proxTopic 		<< "/" << GetId() << "/proximity";
	positionTopic 	<< "/" << GetId() << "/pose";
	rabTopic 		<< "/" << GetId() << "/rab";
	rabDataTopic	<< "/" << GetId() << "/rab_data";
	lightListPublisher_ = ArgosRosFootbot::nodeHandle -> create_publisher<LightList>(lightTopic.str(), 1);
	blobPublisher_ 		= ArgosRosFootbot::nodeHandle -> create_publisher<BlobList>(blobTopic.str(), 1);
	promixityPublisher_ = ArgosRosFootbot::nodeHandle -> create_publisher<ProximityList>(proxTopic.str(), 1);
	positionPublisher_ 	= ArgosRosFootbot::nodeHandle -> create_publisher<geometry_msgs::msg::PoseStamped>(positionTopic.str(), 1);
	rabPublisher_ 		= ArgosRosFootbot::nodeHandle -> create_publisher<PacketList>(rabTopic.str(), 1);
	rabDataPublisher_ 	= ArgosRosFootbot::nodeHandle -> create_publisher<std_msgs::msg::Float64MultiArray>(rabDataTopic.str(), 1);

	/*********************************
	 * Create subscribers
	 ********************************/
	stringstream cmdVelTopic, cmdRabTopic, cmdLedTopic;
	cmdVelTopic 	<< "/" << GetId() << "/cmd_vel";
	cmdRabTopic		<< "/" << GetId() << "/cmd_rab";
	cmdLedTopic		<< "/" << GetId() << "/cmd_led";
	cmdVelSubscriber_ = ArgosRosFootbot::nodeHandle -> create_subscription<Twist>(
						cmdVelTopic.str(),
						1,
						std::bind(&ArgosRosFootbot::cmdVelCallback, this, _1)
						);
	cmdRabSubscriber_ = ArgosRosFootbot::nodeHandle -> create_subscription<Packet>(
						cmdRabTopic.str(),
						1,
						std::bind(&ArgosRosFootbot::cmdRabCallback, this, _1)
						);
	cmdLedSubscriber_ = ArgosRosFootbot::nodeHandle -> create_subscription<Led>(
						cmdLedTopic.str(),
						1,
						std::bind(&ArgosRosFootbot::cmdLedCallback, this, _1)
						);
	/********************************
	 * Get sensor/actuator handles
	 ********************************/
	m_pcLight  		= GetSensor < CCI_FootBotLightSensor					>("footbot_light");
	m_pcProximity 	= GetSensor < CCI_FootBotProximitySensor				>("footbot_proximity");
	m_pcLEDs   		= GetActuator<CCI_LEDsActuator                          >("leds");
	m_pcCamera 		= GetSensor < CCI_ColoredBlobOmnidirectionalCameraSensor>("colored_blob_omnidirectional_camera");
	m_pcPosition 	= GetSensor < CCI_PositioningSensor						>("positioning");
	m_pcRABS 		= GetSensor < CCI_RangeAndBearingSensor					>("range_and_bearing" );

	/********************************
	 * Get actuator handles
	 ********************************/
	m_pcWheels = GetActuator< CCI_DifferentialSteeringActuator >("differential_steering");
	m_pcRABA = GetActuator< CCI_RangeAndBearingActuator        >("range_and_bearing" );

	/*
	* Other init stuff
	*/
	Reset();
	/*
	* Parse the configuration file
	*
	* The user defines this part. Here, the algorithm accepts three
	* parameters and it's nice to put them in the config file so we don't
	* have to recompile if we want to try other settings.
	*/
	GetNodeAttributeOrDefault(t_node, "stopWithoutSubscriberCount", stopWithoutSubscriberCount, stopWithoutSubscriberCount);

}

bool blobComparator(Blob a, Blob b) {
	return a.angle < b.angle;
}

void ArgosRosFootbot::ControlStep() {

	rclcpp::spin_some(ArgosRosFootbot::nodeHandle);

	/*********************************
	 * Get readings from light sensor
	 *********************************/
	const CCI_FootBotLightSensor::TReadings& tLightReads = m_pcLight->GetReadings();
	LightList lightList;
	lightList.n = tLightReads.size();
	for (size_t i = 0; i < lightList.n; ++i) {
		Light light;
		light.value = tLightReads[i].Value;
		light.angle = tLightReads[i].Angle.GetValue();
		lightList.lights.push_back(light);

		//cout << GetId() << ": value: " << light.value << ": angle: " << light.angle << endl;
	}

	lightListPublisher_ -> publish(lightList);

	/***********************************
	 * Get readings from proximity sensor
	 ***********************************/
	const CCI_FootBotProximitySensor::TReadings& tProxReads = m_pcProximity->GetReadings();
	ProximityList proxList;
	proxList.n = tProxReads.size();
	for (size_t i = 0; i < proxList.n; ++i) {
		Proximity prox;
		prox.value = tProxReads[i].Value;
		prox.angle = tProxReads[i].Angle.GetValue();
		proxList.proximities.push_back(prox);

		//cout << GetId() << ": value: " << prox.value << ": angle: " << prox.angle << endl;
	}

	promixityPublisher_ -> publish(proxList);

	/**************************************************************
	 * Get readings from Colored Blob Omnidirectional Camera Sensor
	 *************************************************************/
	const CCI_ColoredBlobOmnidirectionalCameraSensor::SReadings& camReads = m_pcCamera->GetReadings();
	BlobList blobList;
	blobList.n = camReads.BlobList.size();
	Blob blob;
	for (size_t i = 0; i < blobList.n; ++i) {
		//Blob blob;
		stringstream ss;
		ss << camReads.BlobList[i]->Color;
		blob.color = ss.str();
		blob.distance = camReads.BlobList[i]->Distance;

		// Make the angle of the puck in the range [-PI, PI].  This is useful for
		// tasks such as homing in on a puck using a simple controller based on
		// the sign of this angle.
		blob.angle = camReads.BlobList[i]->Angle.GetValue();//.SignedNormalize().GetValue();
		blobList.blobs.push_back(blob);
		//std::cout << "value: " << blob.distance << ": angle: " << blob.angle << " color: " << blob.color << std::endl;

	}

	// Sort the puck list by angle.  This is useful for the purposes of extracting meaning from
	// the local puck configuration (e.g. fitting a lines to the detected pucks).
	sort(blobList.blobs.begin(), blobList.blobs.end(), blobComparator);

	blobPublisher_ -> publish(blobList);

	/*********************************************************************
	 * Get readings from Positioning sensor
	 * TODO: Find an elegant way to make assignment
	 * Problem: can't directly assign argos::CVector3 to geometry::Vector3
	 * Same with the Quaternion
	 **********************************************************************/
	const CCI_PositioningSensor::SReading& tPosReads = m_pcPosition->GetReading();
	
	geometry_msgs::msg::PoseStamped bot_pose;
	bot_pose.header.frame_id = "map";
	bot_pose.header.stamp = rclcpp::Clock().now();

	bot_pose.pose.position.x = tPosReads.Position.GetX();
	bot_pose.pose.position.y = tPosReads.Position.GetY();
	bot_pose.pose.position.z = tPosReads.Position.GetZ();

	bot_pose.pose.orientation.w = tPosReads.Orientation.GetW();
	bot_pose.pose.orientation.x = tPosReads.Orientation.GetX();
	bot_pose.pose.orientation.y = tPosReads.Orientation.GetY();
	bot_pose.pose.orientation.z = tPosReads.Orientation.GetZ();

	positionPublisher_ -> publish(bot_pose);
	/*********************************************
	 * send message via Range-And-Bearing-Actuator
	 *********************************************/
	
	Rab_actuator(bot_pose);

	/*********************************************
	 * Get readings from Range-And-Bearing-Sensor
	 *********************************************/
	const CCI_RangeAndBearingSensor::TReadings& tRabReads = m_pcRABS->GetReadings();
	std_msgs::msg::Float64MultiArray multiple_pose;
	PacketList packetList;
	packetList.n = tRabReads.size();
	// cout << GetId() << ": received the following broadcasts: " << packetList.n << endl;

	//  准备 Float64MultiArray 的布局
	multiple_pose.layout.dim.resize(1 + 7); // 1维用于消息数量，7维用于数据字段标签

	multiple_pose.layout.dim[0].label = "messages"; // 消息维度
	multiple_pose.layout.dim[0].size = tRabReads.size();
	multiple_pose.layout.dim[0].stride = tRabReads.size() * 7; // 总共 7 个数据字段

	// 为每个数据字段添加标签
	std::vector<std::string> labels = {"x", "y", "z", "o_w", "o_x", "o_y", "o_z"};
	for (size_t j = 0; j < 7; ++j)
	{
		multiple_pose.layout.dim[j + 1].label = labels[j];
		multiple_pose.layout.dim[j + 1].size = 1;
		multiple_pose.layout.dim[j + 1].stride = 1;
	}

	multiple_pose.layout.data_offset = 0;

	// 为 multiple_pose 数据分配足够的空间
	multiple_pose.data.resize(packetList.n * 7);

	for (size_t i = 0; i < packetList.n; ++i) {
		Packet packet;
		packet.range = tRabReads[i].Range;
		packet.h_bearing = tRabReads[i].HorizontalBearing.GetValue();
		packet.v_bearing = tRabReads[i].VerticalBearing.GetValue();

		packetList.packets.push_back(packet);

		// 从 CByteArray 中解析数据并填充 multiple_pose.data
		CByteArray cBuf = tRabReads[i].Data;

		float x, y, z, o_w, o_x, o_y, o_z;
		cBuf >> x >> y >> z >> o_w >> o_x >> o_y >> o_z;

		// 将解析后的数据放入 multiple_pose 的 data 数组中
		multiple_pose.data[i * 7] = static_cast<double>(x);
		multiple_pose.data[i * 7 + 1] = static_cast<double>(y);
		multiple_pose.data[i * 7 + 2] = static_cast<double>(z);
		multiple_pose.data[i * 7 + 3] = static_cast<double>(o_w);
		multiple_pose.data[i * 7 + 4] = static_cast<double>(o_x);
		multiple_pose.data[i * 7 + 5] = static_cast<double>(o_y);
		multiple_pose.data[i * 7 + 6] = static_cast<double>(o_z);
	}

	rabPublisher_->publish(packetList);
	rabDataPublisher_->publish(multiple_pose);

	/*********************************************
	 * receive message from ros and send it to Left-Right-wheel-Actuator
	 *********************************************/
	// If we haven't heard from the subscriber in a while, set the speed to zero.
	if (stepsSinceCallback > stopWithoutSubscriberCount) {
		leftSpeed = 0;
		rightSpeed = 0;
	} else {
		stepsSinceCallback++;
	}
	//std::cout << GetId() << ": left-wheel: " << leftSpeed << ": right-wheel: " << rightSpeed << std::endl;
	m_pcWheels->SetLinearVelocity(leftSpeed, rightSpeed);
}

void ArgosRosFootbot::Reset() {
   /* Enable camera filtering */
   m_pcCamera->Enable();
   /** Set the LED intensity first */
   m_pcLEDs -> SetAllIntensities( 100 );
   /* Set beacon color to all red to be visible for other robots */
   m_pcLEDs->SetAllColors(CColor::RED);
}

void ArgosRosFootbot::cmdVelCallback(const Twist& twist) {
	//cout << "cmdVelCallback: " << GetId() << endl;
    
    double v = twist.linear.x;       // Forward linear velocity
    double omega = twist.angular.z;  // Rotational (angular) velocity
    double L = HALF_BASELINE * 2;    // Distance between wheels (wheelbase)
    double R = WHEEL_RADIUS;         // Wheel radius

    // Calculate left and right wheel speeds using differential drive kinematics
    leftSpeed = (v - (L / 2) * omega) / R;
    rightSpeed = (v + (L / 2) * omega) / R;

	stepsSinceCallback = 0;
}

void ArgosRosFootbot::cmdRabCallback(const Packet& packet){
	//cout << GetId() << " Packet data as received: " << packet.data[0] << " for id: " << std::stoi( packet.id ) <<endl;
	m_pcRABA -> SetData(0, packet.data[0]);
	m_pcRABA -> SetData(1, std::stoi( packet.id ));
}
void ArgosRosFootbot::cmdLedCallback(const Led& ledColor){
	//cout << " Received the following color: " << ledColor.color << std::endl;
	if ( ledColor.color == "yellow" ){
		m_pcLEDs->SetAllColors(CColor::ORANGE);
		//cout << GetId() << " setting leds to yellow." << std::endl;
	}
	else if ( ledColor.color == "green" ){
		m_pcLEDs->SetAllColors(CColor::CYAN);
		//cout << GetId() << " setting leds to green." << std::endl;
	}
}

void ArgosRosFootbot::Rab_actuator(const geometry_msgs::msg::PoseStamped& pose){
	
	CByteArray cBuf;
	float x = pose.pose.position.x;
	float y = pose.pose.position.y;
	float z = pose.pose.position.z;

	float o_w = pose.pose.orientation.w;
	float o_x = pose.pose.orientation.x;
	float o_y = pose.pose.orientation.y;
	float o_z = pose.pose.orientation.z;

	cBuf << x;
	cBuf << y;
	cBuf << z;

	cBuf << o_w;
	cBuf << o_x;
	cBuf << o_y;
	cBuf << o_z;

	size_t SIZE = 28;
	while (cBuf.Size() < SIZE)
	{
		cBuf << '\0'; 
	}

	m_pcRABA->SetData(cBuf);
}
/*
* This statement notifies ARGoS of the existence of the controller.
* It binds the class passed as first argument to the string passed as
* second argument.
* The string is then usable in the configuration file to refer to this
* controller.
 * When ARGoS reads that string in the configuration file, it knows which
 * controller class to instantiate.
 * See also the configuration files for an example of how this is used.
 */
REGISTER_CONTROLLER(ArgosRosFootbot, "argos_ros_bot_controller")
