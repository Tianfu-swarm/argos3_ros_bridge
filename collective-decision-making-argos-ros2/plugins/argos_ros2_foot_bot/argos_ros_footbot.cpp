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
std::shared_ptr<rclcpp::Node> initNode()
{
	int argc = 1;
	char *argv = (char *)"";
	if (rclcpp::get_contexts().empty())
	{
		rclcpp::init(argc, &argv);
	}

	return std::make_shared<rclcpp::Node>("argos_ros_node");
}

std::shared_ptr<rclcpp::Node> ArgosRosFootbot::nodeHandle = initNode();

ArgosRosFootbot::ArgosRosFootbot() : m_pcWheels(NULL),
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
									 rightSpeed(0) {}

ArgosRosFootbot::~ArgosRosFootbot() {}

void ArgosRosFootbot::Init(TConfigurationNode &t_node)
{
	/********************************
	 * Create the topics to publish
	 *******************************/
	stringstream lightTopic, blobTopic, proxTopic, positionTopic, rabDataTopic, rabPointTopic;
	lightTopic << "/" << GetId() << "/light";
	blobTopic << "/" << GetId() << "/blob";
	proxTopic << "/" << GetId() << "/proximity_point";
	positionTopic << "/" << GetId() << "/pose";
	rabDataTopic << "/" << GetId() << "/rab_sensor";
	rabPointTopic << "/" << GetId() << "/rab_point";

	lightListPublisher_ = ArgosRosFootbot::nodeHandle->create_publisher<LightList>(lightTopic.str(), 1);
	blobPublisher_ = ArgosRosFootbot::nodeHandle->create_publisher<BlobList>(blobTopic.str(), 1);
	promixityPublisher_ = ArgosRosFootbot::nodeHandle->create_publisher<sensor_msgs::msg::PointCloud2>(proxTopic.str(), 1);
	positionPublisher_ = ArgosRosFootbot::nodeHandle->create_publisher<geometry_msgs::msg::PoseStamped>(positionTopic.str(), 1);
	rabDataPublisher_ = ArgosRosFootbot::nodeHandle->create_publisher<std_msgs::msg::Float64MultiArray>(rabDataTopic.str(), 1);
	rabPointPublisher_ = ArgosRosFootbot::nodeHandle->create_publisher<sensor_msgs::msg::PointCloud2>(rabPointTopic.str(), 1);

	/*********************************
	 * Create subscribers
	 ********************************/
	stringstream cmdVelTopic, cmdRabTopic, cmdLedTopic;
	cmdVelTopic << "/" << GetId() << "/cmd_vel";
	cmdRabTopic << "/" << GetId() << "/rab_actuator";
	cmdLedTopic << "/" << GetId() << "/cmd_led";
	cmdVelSubscriber_ = ArgosRosFootbot::nodeHandle->create_subscription<Twist>(
		cmdVelTopic.str(),
		1,
		std::bind(&ArgosRosFootbot::cmdVelCallback, this, _1));
	cmdRabSubscriber_ = ArgosRosFootbot::nodeHandle->create_subscription<std_msgs::msg::Float64MultiArray>(
		cmdRabTopic.str(),
		1,
		std::bind(&ArgosRosFootbot::cmdRabCallback, this, _1));
	cmdLedSubscriber_ = ArgosRosFootbot::nodeHandle->create_subscription<Led>(
		cmdLedTopic.str(),
		1,
		std::bind(&ArgosRosFootbot::cmdLedCallback, this, _1));
	/********************************
	 * Get sensor/actuator handles
	 ********************************/
	m_pcLight = GetSensor<CCI_FootBotLightSensor>("footbot_light");
	m_pcProximity = GetSensor<CCI_FootBotProximitySensor>("footbot_proximity");
	m_pcLEDs = GetActuator<CCI_LEDsActuator>("leds");
	m_pcCamera = GetSensor<CCI_ColoredBlobOmnidirectionalCameraSensor>("colored_blob_omnidirectional_camera");
	m_pcPosition = GetSensor<CCI_PositioningSensor>("positioning");
	m_pcRABS = GetSensor<CCI_RangeAndBearingSensor>("range_and_bearing");

	/********************************
	 * Get actuator handles
	 ********************************/
	m_pcWheels = GetActuator<CCI_DifferentialSteeringActuator>("differential_steering");
	m_pcRABA = GetActuator<CCI_RangeAndBearingActuator>("range_and_bearing");

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

bool blobComparator(Blob a, Blob b)
{
	return a.angle < b.angle;
}

void ArgosRosFootbot::ControlStep()
{

	rclcpp::spin_some(ArgosRosFootbot::nodeHandle);

	/*********************************
	 * Get readings from light sensor
	 *********************************/
	const CCI_FootBotLightSensor::TReadings &tLightReads = m_pcLight->GetReadings();
	LightList lightList;
	lightList.n = tLightReads.size();
	for (size_t i = 0; i < lightList.n; ++i)
	{
		Light light;
		light.value = tLightReads[i].Value;
		light.angle = tLightReads[i].Angle.GetValue();
		lightList.lights.push_back(light);

		// cout << GetId() << ": value: " << light.value << ": angle: " << light.angle << endl;
	}

	lightListPublisher_->publish(lightList);

	/***********************************
	 * Get readings from proximity sensor
	 ***********************************/
	const CCI_FootBotProximitySensor::TReadings &tProxReads = m_pcProximity->GetReadings();
	sensor_msgs::msg::PointCloud2 Proximity_point;

	// 初始化 PointCloud2 消息
	Proximity_point.header.stamp = rclcpp::Clock().now(); // 设置时间戳
	std::stringstream Proximity_frame_id;
	Proximity_frame_id << GetId() << "/Proximity_sensor";
	Proximity_point.header.frame_id = Proximity_frame_id.str(); // 设置帧 ID (根据需要修改)

	// 创建并修改 PointCloud2 消息
	sensor_msgs::PointCloud2Modifier proximity_modifier(Proximity_point);
	proximity_modifier.setPointCloud2FieldsByString(1, "xyz"); // 设置为包含 xyz 字段
	proximity_modifier.resize(tProxReads.size());			   // 调整点云大小

	// 获取点云迭代器

	sensor_msgs::PointCloud2Iterator<float> proximity_iter_x(Proximity_point, "x");
	sensor_msgs::PointCloud2Iterator<float> proximity_iter_y(Proximity_point, "y");
	sensor_msgs::PointCloud2Iterator<float> proximity_iter_z(Proximity_point, "z");

	// 假设已知衰减系数 λ
	double lambda = 1.0;

	for (size_t i = 0; i < tProxReads.size(); ++i)
	{

		if (tProxReads[i].Value == 0)
		{
			continue;
		}

		double range = -std::log(tProxReads[i].Value) / lambda; // 计算实际距离
		double angle = tProxReads[i].Angle.GetValue();			// 获取角度（单位：弧度）

		// 将距离和角度转换为笛卡尔坐标
		*proximity_iter_x = range * cos(angle); // 计算 x 坐标
		*proximity_iter_y = range * sin(angle); // 计算 y 坐标
		*proximity_iter_z = 0.0;				// 如果是2D平面，z 设置为 0

		++proximity_iter_x;
		++proximity_iter_y;
		++proximity_iter_z;
	}

	// 发布点云消息
	promixityPublisher_->publish(Proximity_point);

	/**************************************************************
	 * Get readings from Colored Blob Omnidirectional Camera Sensor
	 *************************************************************/
	const CCI_ColoredBlobOmnidirectionalCameraSensor::SReadings &camReads = m_pcCamera->GetReadings();
	BlobList blobList;
	blobList.n = camReads.BlobList.size();
	Blob blob;
	for (size_t i = 0; i < blobList.n; ++i)
	{
		// Blob blob;
		stringstream ss;
		ss << camReads.BlobList[i]->Color;
		blob.color = ss.str();
		blob.distance = camReads.BlobList[i]->Distance;

		// Make the angle of the puck in the range [-PI, PI].  This is useful for
		// tasks such as homing in on a puck using a simple controller based on
		// the sign of this angle.
		blob.angle = camReads.BlobList[i]->Angle.GetValue(); //.SignedNormalize().GetValue();
		blobList.blobs.push_back(blob);
		// std::cout << "value: " << blob.distance << ": angle: " << blob.angle << " color: " << blob.color << std::endl;
	}

	// Sort the puck list by angle.  This is useful for the purposes of extracting meaning from
	// the local puck configuration (e.g. fitting a lines to the detected pucks).
	sort(blobList.blobs.begin(), blobList.blobs.end(), blobComparator);

	blobPublisher_->publish(blobList);

	/*********************************************************************
	 * Get readings from Positioning sensor
	 * TODO: Find an elegant way to make assignment
	 * Problem: can't directly assign argos::CVector3 to geometry::Vector3
	 * Same with the Quaternion
	 **********************************************************************/
	const CCI_PositioningSensor::SReading &tPosReads = m_pcPosition->GetReading();

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

	positionPublisher_->publish(bot_pose);

	/*********************************************
	 * Get readings from Range-And-Bearing-Sensor
	 *********************************************/
	const CCI_RangeAndBearingSensor::TReadings &tRabReads = m_pcRABS->GetReadings();
	std_msgs::msg::Float64MultiArray rabSensorData;
	sensor_msgs::msg::PointCloud2 rabPointData;
	PacketList packetList;
	packetList.n = tRabReads.size();
	// 设置头信息
	rabPointData.header.stamp = rclcpp::Clock().now();
	stringstream frame_id;
	frame_id << GetId() << "/rab_sensor";
	rabPointData.header.frame_id = frame_id.str();
	rabPointData.height = 1; // 无组织点云
	rabPointData.width = packetList.n;
	rabPointData.is_dense = true;

	// 定义字段
	sensor_msgs::PointCloud2Modifier modifier(rabPointData);
	modifier.setPointCloud2FieldsByString(1, "xyz");
	modifier.resize(packetList.n);

	// 填充点云数据
	sensor_msgs::PointCloud2Iterator<float> iter_x(rabPointData, "x");
	sensor_msgs::PointCloud2Iterator<float> iter_y(rabPointData, "y");
	sensor_msgs::PointCloud2Iterator<float> iter_z(rabPointData, "z");
	// cout << GetId() << ": received the following broadcasts: " << packetList.n << endl;
	for (size_t i = 0; i < packetList.n; ++i)
	{
		Packet packet;
		packet.range = tRabReads[i].Range;
		packet.h_bearing = tRabReads[i].HorizontalBearing.GetValue();
		packet.v_bearing = tRabReads[i].VerticalBearing.GetValue();

		packetList.packets.push_back(packet);

		//////to pointcloud2 type
		double range = tRabReads[i].Range / 100;
		double h_bearing = tRabReads[i].HorizontalBearing.GetValue(); // 水平角度
		double v_bearing = tRabReads[i].VerticalBearing.GetValue();	  // 垂直角度

		// 转换为笛卡尔坐标
		*iter_x = range * std::cos(v_bearing) * std::cos(h_bearing);
		*iter_y = range * std::cos(v_bearing) * std::sin(h_bearing);
		*iter_z = range * std::sin(v_bearing);

		// 迭代到下一个点
		++iter_x;
		++iter_y;
		++iter_z;

		CByteArray cBuf_copy;
		cBuf_copy = tRabReads[i].Data;
		double extractedValue;
		while (cBuf_copy.Size() >= sizeof(double))
		{ // 每次提取一个 double
			cBuf_copy >> extractedValue;
			rabSensorData.data.push_back(extractedValue);
		}
	}

	rabPointPublisher_->publish(rabPointData);

	rabDataPublisher_->publish(rabSensorData);

	/*********************************************
	 * receive message from ros and send it to Left-Right-wheel-Actuator
	 *********************************************/
	// If we haven't heard from the subscriber in a while, set the speed to zero.
	if (stepsSinceCallback > stopWithoutSubscriberCount)
	{
		leftSpeed = 0;
		rightSpeed = 0;
	}
	else
	{
		stepsSinceCallback++;
	}
	// std::cout << GetId() << ": left-wheel: " << leftSpeed << ": right-wheel: " << rightSpeed << std::endl;
	m_pcWheels->SetLinearVelocity(leftSpeed, rightSpeed);
}

void ArgosRosFootbot::Reset()
{
	/* Enable camera filtering */
	m_pcCamera->Enable();
	/** Set the LED intensity first */
	m_pcLEDs->SetAllIntensities(100);
	/* Set beacon color to all red to be visible for other robots */
	m_pcLEDs->SetAllColors(CColor::RED);
}

void ArgosRosFootbot::cmdVelCallback(const Twist &twist)
{
	// cout << "cmdVelCallback: " << GetId() << endl;

	double v = twist.linear.x;		// Forward linear velocity
	double omega = twist.angular.z; // Rotational (angular) velocity
	double L = HALF_BASELINE * 2;	// Distance between wheels (wheelbase)
	double R = WHEEL_RADIUS;		// Wheel radius

	// Calculate left and right wheel speeds using differential drive kinematics
	leftSpeed = (v - (L / 2) * omega) / R;
	rightSpeed = (v + (L / 2) * omega) / R;

	stepsSinceCallback = 0;
}

void ArgosRosFootbot::cmdRabCallback(const std_msgs::msg::Float64MultiArray &rabActuator)
{

	CByteArray cBuf;

	for (size_t i = 0; i < rabActuator.data.size(); i++)
	{
		double data = rabActuator.data[i];
		cBuf << data;
		std::cout << "get rab_actuator message from ros,value is :   " << rabActuator.data[i] << std::endl;
	}
	size_t SIZE = 120;
	while (cBuf.Size() < SIZE)
	{
		double padding = 0.0;
		cBuf << padding;
	}
	m_pcRABA->SetData(cBuf);
}

void ArgosRosFootbot::cmdLedCallback(const Led &ledColor)
{
	// cout << " Received the following color: " << ledColor.color << std::endl;
	if (ledColor.color == "yellow")
	{
		m_pcLEDs->SetAllColors(CColor::ORANGE);
		// cout << GetId() << " setting leds to yellow." << std::endl;
	}
	else if (ledColor.color == "green")
	{
		m_pcLEDs->SetAllColors(CColor::CYAN);
		// cout << GetId() << " setting leds to green." << std::endl;
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
