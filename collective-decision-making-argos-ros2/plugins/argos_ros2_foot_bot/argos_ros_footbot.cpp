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
	stringstream lightTopic, blobTopic, proxTopic, positionTopic, rabTopic;
	lightTopic 		<< "/" << GetId() << "/light";
	blobTopic 		<< "/" << GetId() << "/blob";
	proxTopic 		<< "/" << GetId() << "/proximity";
	positionTopic 	<< "/" << GetId() << "/pose";
	rabTopic 		<< "/" << GetId() << "/rab_sensor";
	
	lightListPublisher_ = ArgosRosFootbot::nodeHandle -> create_publisher<LightList>(lightTopic.str(), 1);
	blobPublisher_ 		= ArgosRosFootbot::nodeHandle -> create_publisher<BlobList>(blobTopic.str(), 1);
	promixityPublisher_ = ArgosRosFootbot::nodeHandle -> create_publisher<ProximityList>(proxTopic.str(), 1);
	positionPublisher_ 	= ArgosRosFootbot::nodeHandle -> create_publisher<geometry_msgs::msg::PoseStamped>(positionTopic.str(), 1);
	rabPublisher_ 		= ArgosRosFootbot::nodeHandle -> create_publisher<std_msgs::msg::Float64MultiArray>(rabTopic.str(), 1);
	

	/*********************************
	 * Create subscribers
	 ********************************/
	stringstream cmdVelTopic, cmdRabTopic, cmdLedTopic;
	cmdVelTopic 	<< "/" << GetId() << "/cmd_vel";
	cmdRabTopic		<< "/" << GetId() << "/rab_actuator";
	cmdLedTopic		<< "/" << GetId() << "/cmd_led";
	cmdVelSubscriber_ = ArgosRosFootbot::nodeHandle -> create_subscription<Twist>(
						cmdVelTopic.str(),
						1,
						std::bind(&ArgosRosFootbot::cmdVelCallback, this, _1)
						);
	cmdRabSubscriber_ = ArgosRosFootbot::nodeHandle -> create_subscription<std_msgs::msg::Float64MultiArray>(
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
	 * Get readings from Range-And-Bearing-Sensor
	 *********************************************/
	const CCI_RangeAndBearingSensor::TReadings& tRabReads = m_pcRABS->GetReadings();
	std_msgs::msg::Float64MultiArray rabSensorData;
	PacketList packetList;
	packetList.n = tRabReads.size();
	// cout << GetId() << ": received the following broadcasts: " << packetList.n << endl;
	for (size_t i = 0; i < packetList.n; ++i)
	{
		Packet packet;
		packet.range = tRabReads[i].Range;
		packet.h_bearing = tRabReads[i].HorizontalBearing.GetValue();
		packet.v_bearing = tRabReads[i].VerticalBearing.GetValue();

		packetList.packets.push_back(packet);

		const UInt8 *byteData = tRabReads[i].Data.ToCArray();			 // 获取字节数据指针
		size_t num_elements = tRabReads[i].Data.Size() / sizeof(double); // 计算 double 的数量

		// 如果数据指针有效
		if (byteData != nullptr)
		{
			// 遍历字节数据，并将每 8 个字节转换为一个 double
			for (size_t j = 0; j < num_elements; ++j)
			{
				double value;

				// 使用 memcpy 读取 8 个字节并转换为 double
				std::memcpy(&value, byteData + j * sizeof(double), sizeof(double));

				rabSensorData.data.push_back(value);
				//std::cout << "get rab_sensor message ,value is :   " << value << std::endl;
			}
		}
	}

	//rabPublisher_->publish(packetList);
	rabPublisher_->publish(rabSensorData);

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

void ArgosRosFootbot::cmdRabCallback(const std_msgs::msg::Float64MultiArray &rabActuator)
{

	CByteArray cBuf;

	for (size_t i; i < rabActuator.data.size(); i++)
	{
		float data = rabActuator.data[i];
		cBuf << data;
		std::cout << "get rab_actuator message from ros,value is :   " << rabActuator.data[i] << std::endl;
	}
	size_t SIZE = 120;
	while (cBuf.Size() < SIZE)
	{
		cBuf << '\0';
	}

	const UInt8 *byteData = cBuf.ToCArray();
	size_t dataSize = cBuf.Size();

	// 确保数据有效
	if (byteData != nullptr)
    {
        std::cout << "Printing double values from CByteArray: " << std::endl;

        // 解析每8个字节为一个 double
        for (size_t i = 0; i < dataSize / sizeof(float); ++i)
        {
            double value;
            std::memcpy(&value, byteData + i * sizeof(float), sizeof(float));
            std::cout << "Double value " << i + 1 << ": " << value << std::endl;
        }
    }

	m_pcRABA->SetData(cBuf);
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
