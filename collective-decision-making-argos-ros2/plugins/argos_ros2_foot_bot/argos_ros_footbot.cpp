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
	rabDataTopic	<< "/" << GetId() << "/rabData";
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
	 * The position.x part data is：
	 * [0]:polarity
	 * [1]:Integer Part
	 * [2]:Decimal Part
	 * The position.y part data is：
	 * [3]:polarity
	 * [4]:Integer Part
	 * [5]:Decimal Part
	 * The orientation.z part data is：
	 * [6]:polarity
	 * [7]:Integer Part
	 * [8]:Decimal Part
	 * The orientation.w part data is：
	 * [9]:polarity
	 * [10]:Integer Part
	 * [11]:Decimal Part
	 * The robot moves only in the x, y plane. Only the yaw angle changes.
	 *********************************************/
	//Rab_actuator_encode(bot_pose);
	Rab_actuator(bot_pose);



	
	/*********************************************
	 * Get readings from Range-And-Bearing-Sensor
	 *********************************************/
	const CCI_RangeAndBearingSensor::TReadings& tRabReads = m_pcRABS->GetReadings();
	std_msgs::msg::Float64MultiArray multiple_pose;
	PacketList packetList;
	packetList.n = tRabReads.size();
	//cout << GetId() << ": received the following broadcasts: " << packetList.n << endl;
	for (size_t i = 0; i < packetList.n; ++i) {
		Packet packet;
		packet.range = tRabReads[i].Range;
		packet.h_bearing = tRabReads[i].HorizontalBearing.GetValue();
		packet.v_bearing = tRabReads[i].VerticalBearing.GetValue();

		
		packet.data.push_back(tRabReads[i].Data[0]);
		packet.data.push_back(tRabReads[i].Data[1]);

		packetList.packets.push_back(packet);

		// std::vector<uint8_t> tRabRead;
		// for (size_t j = 0; j < 12; j++)
		// {
		// 	tRabRead.push_back(tRabReads[i].Data[j]);
		// }
		// std::vector<double> pose;
		// pose = Rab_actuator_decode(tRabRead);

		// multiple_pose.data.insert(multiple_pose.data.end(), pose.begin(), pose.end());
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
/*********************************************
 * Rab Actuator encode and decode
 * The position.x part data is：
 * [0]:polarity
 * [1]:Integer Part
 * [2]:Decimal Part
 * The position.y part data is：
 * [3]:polarity
 * [4]:Integer Part
 * [5]:Decimal Part
 * The orientation.z part data is：
 * [6]:polarity
 * [7]:Integer Part
 * [8]:Decimal Part
 * The orientation.w part data is：
 * [9]:polarity
 * [10]:Integer Part
 * [11]:Decimal Part
 * The robot moves only in the x, y plane. Only the yaw angle changes.
 *********************************************/
void ArgosRosFootbot::Rab_actuator_encode(const geometry_msgs::msg::PoseStamped &pose)
{
	m_pcRABA->ClearData();
	uint8_t polarity_position_x;	// 0-positive, 1-negative
	uint8_t polarity_position_y;	// 0-positive, 1-negative
	uint8_t polarity_orientation_z; // 0-positive, 1-negative
	uint8_t polarity_orientation_w; // 0-positive, 1-negative

	uint8_t integer_position_x;
	uint8_t decimal_position_x;
	uint8_t integer_position_y;
	uint8_t decimal_position_y;
	uint8_t integer_orientation_z;
	uint8_t decimal_orientation_z;
	uint8_t integer_orientation_w;
	uint8_t decimal_orientation_w;

	std::vector<double> result_position_x;
	result_position_x = encode(pose.pose.position.x);
	polarity_position_x = result_position_x[0];
	integer_position_x = result_position_x[1];
	decimal_position_x = result_position_x[2];

	std::vector<double> result_position_y;
	result_position_y = encode(pose.pose.position.y);
	polarity_position_y = result_position_y[0];
	integer_position_y = result_position_y[1];
	decimal_position_y = result_position_y[2];

	std::vector<double> result_orientation_z;
	result_orientation_z = encode(pose.pose.orientation.z);
	polarity_orientation_z = result_orientation_z[0];
	integer_orientation_z = result_orientation_z[1];
	decimal_orientation_z = result_orientation_z[2];

	std::vector<double> result_orientation_w;
	result_orientation_w = encode(pose.pose.orientation.w);
	polarity_orientation_w = result_orientation_w[0];
	integer_orientation_w = result_orientation_w[1];
	decimal_orientation_w = result_orientation_w[2];

	m_pcRABA->SetData(0, polarity_position_x);
	m_pcRABA->SetData(1, integer_position_x);
	m_pcRABA->SetData(2, decimal_position_x);
	m_pcRABA->SetData(3, polarity_position_y);
	m_pcRABA->SetData(4, integer_position_y);
	m_pcRABA->SetData(5, decimal_position_y);
	m_pcRABA->SetData(6, polarity_orientation_z);
	m_pcRABA->SetData(7, integer_orientation_z);
	m_pcRABA->SetData(8, decimal_orientation_z);
	m_pcRABA->SetData(9, polarity_orientation_w);
	m_pcRABA->SetData(10, integer_orientation_w);
	m_pcRABA->SetData(11, decimal_orientation_w);
}
std::vector<double> ArgosRosFootbot::encode(double num)
{
	std::vector<double> result;
	uint8_t polarity;
	uint8_t integer;
	uint8_t decimal;
	if (num > 0)
	{
		polarity = 0;
		integer = trunc(num);
		if (num > 255)
		{
			std::cout << "Robot's position exceeds the maximum value (255, 255)" << std::endl;
		}
		decimal = num - integer;
		if (decimal > 0.255)
		{
			decimal = decimal * 100;
		}
		else
		{
			decimal = decimal * 1000;
		}
	}
	else
	{
		polarity = 1;
		integer = -trunc(num);
		if (num > 255)
		{
			std::cout << "Robot's position exceeds the maximum value (255, 255)" << std::endl;
		}
		decimal = -(num + integer);
		if (decimal > 0.255)
		{
			decimal = decimal * 1000;
		}
		else
		{
			decimal = decimal * 100;
		}
	}
	result.push_back(polarity);
	result.push_back(integer);
	result.push_back(decimal);

	return result;
}
std::vector<double> ArgosRosFootbot::Rab_actuator_decode(std::vector<uint8_t> data)
{
	std::vector<double> result;

	double position_x = decode(data[0], data[1], data[2]);
	double position_y = decode(data[3], data[4], data[5]);
	double orientation_z = decode(data[6], data[7], data[8]);
	double orientation_w = decode(data[9], data[10], data[11]);

	result.push_back(position_x);
	result.push_back(position_y);
	result.push_back(orientation_z);
	result.push_back(orientation_w);

	return result;
}
double ArgosRosFootbot::decode(uint8_t polarity, uint8_t integer, uint8_t decimal)
{
	double num;

	// 处理符号
	double sign = (polarity == 0) ? 1.0 : -1.0;

	// 恢复小数部分
	double fractional_part;
	if (decimal > 99) // 如果小数部分超过99，说明它是通过100缩放的
	{
		fractional_part = decimal / 1000.0;
	}
	else // 否则，它是通过1000缩放的
	{
		fractional_part = decimal / 100.0;
	}

	// 还原数字
	num = integer + fractional_part;

	// 应用符号
	return sign * num;
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
