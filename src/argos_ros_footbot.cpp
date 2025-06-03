

/* Include the controller definition */
#include "argos_ros_footbot.h"
using namespace std;
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
									 //  m_pcLight(NULL),
									 m_pcProximity(NULL),
									 m_pcPosition(NULL),
									 m_pcRABA(NULL),
									 m_pcRABS(NULL),
									 //  m_pcSRA(NULL),
									 //  m_pcSRS(NULL),
									 stopWithoutSubscriberCount(10),
									 stepsSinceCallback(0),
									 leftSpeed(0),
									 rightSpeed(0)
{
}

ArgosRosFootbot::~ArgosRosFootbot() {}

void ArgosRosFootbot::Init(TConfigurationNode &t_node)
{
	/********************************
	 * Create the topics to publish
	 *******************************/
	stringstream lightTopic, blobTopic, proxTopic, positionTopic, rabDataTopic, tfTopic, radioTopic;
	// lightTopic << "/" << GetId() << "/light";
	blobTopic << "/" << GetId() << "/blob";
	proxTopic << "/" << GetId() << "/proximity_point";
	positionTopic << "/" << GetId() << "/pose";
	rabDataTopic << "/" << GetId() << "/rab_sensor";
	tfTopic << "/" << GetId() << "/rab_tf";
	radioTopic << "/" << GetId() << "/radio_sensor";

	promixityPublisher_ = ArgosRosFootbot::nodeHandle->create_publisher<sensor_msgs::msg::PointCloud2>(proxTopic.str(), 10);
	positionPublisher_ = ArgosRosFootbot::nodeHandle->create_publisher<geometry_msgs::msg::PoseStamped>(positionTopic.str(), 10);
	rabDataPublisher_ = ArgosRosFootbot::nodeHandle->create_publisher<std_msgs::msg::Float64MultiArray>(rabDataTopic.str(), 10);
	tfPublisher_ = ArgosRosFootbot::nodeHandle->create_publisher<tf2_msgs::msg::TFMessage>(tfTopic.str(), 10);
	radioDataPublisher_ = ArgosRosFootbot::nodeHandle->create_publisher<std_msgs::msg::Float64MultiArray>(radioTopic.str(), 10);

	bool is_clock_publisher = (GetId() == "bot0");
	if (is_clock_publisher)
	{
		clockPublisher_ = ArgosRosFootbot::nodeHandle->create_publisher<rosgraph_msgs::msg::Clock>("/clock", 10);
	}

	/*********************************
	 * Create subscribers
	 ********************************/
	stringstream cmdVelTopic, rabActuatorTopic, radioActuatorTopic;
	cmdVelTopic << "/" << GetId() << "/cmd_vel";
	rabActuatorTopic << "/" << GetId() << "/rab_actuator";
	radioActuatorTopic << "/" << GetId() << "/radio_actuator";

	cmdVelSubscriber_ = ArgosRosFootbot::nodeHandle->create_subscription<Twist>(cmdVelTopic.str(), 10, std::bind(&ArgosRosFootbot::cmdVelCallback, this, _1));
	rabActuatorSubscriber_ = ArgosRosFootbot::nodeHandle->create_subscription<std_msgs::msg::Float64MultiArray>(rabActuatorTopic.str(), 10, std::bind(&ArgosRosFootbot::rabActuatorCallback, this, _1));
	radioActuatorSubscriber_ = ArgosRosFootbot::nodeHandle->create_subscription<std_msgs::msg::Float64MultiArray>(radioActuatorTopic.str(), 10, std::bind(&ArgosRosFootbot::radioActuatorCallback, this, _1));
	/********************************
	 * Get sensor/actuator handles
	 ********************************/
	// m_pcLight = GetSensor<CCI_FootBotLightSensor>("footbot_light");
	m_pcProximity = GetSensor<CCI_FootBotProximitySensor>("footbot_proximity");

	m_pcPosition = GetSensor<CCI_PositioningSensor>("positioning");
	m_pcRABS = GetSensor<CCI_RangeAndBearingSensor>("range_and_bearing");
	m_pcSRS = GetSensor<CCI_SimpleRadiosSensor>("simple_radios");

	/********************************
	 * Get actuator handles
	 ********************************/
	m_pcWheels = GetActuator<CCI_DifferentialSteeringActuator>("differential_steering");
	m_pcRABA = GetActuator<CCI_RangeAndBearingActuator>("range_and_bearing");
	m_pcSRA = GetActuator<CCI_SimpleRadiosActuator>("simple_radios");

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

void ArgosRosFootbot::ControlStep()
{
	const auto &sim = argos::CSimulator::GetInstance();
	argos::CPhysicsEngine &engine = sim.GetPhysicsEngine("dyn2d");
	argos::Real sim_time = sim.GetSpace().GetSimulationClock() * engine.GetPhysicsClockTick();
	rclcpp::Time ros_sim_time(static_cast<uint64_t>(sim_time * 1e9)); // 仿真秒 -> 纳秒

	rosgraph_msgs::msg::Clock clock_msg;
	clock_msg.clock = ros_sim_time;
	if (clockPublisher_)
	{
		clockPublisher_->publish(clock_msg);
	}

	rclcpp::spin_some(ArgosRosFootbot::nodeHandle);

	/***********************************
	 * Get readings from proximity sensor
	 ***********************************/
	const CCI_FootBotProximitySensor::TReadings &tProxReads = m_pcProximity->GetReadings();
	sensor_msgs::msg::PointCloud2 Proximity_point;

	// 初始化 PointCloud2 消息
	Proximity_point.header.stamp = ros_sim_time; // 设置时间戳
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

	/*********************************************************************
	 * Get readings from Positioning sensor
	 **********************************************************************/
	const CCI_PositioningSensor::SReading &tPosReads = m_pcPosition->GetReading();

	geometry_msgs::msg::PoseStamped bot_pose;
	bot_pose.header.frame_id = "map";
	bot_pose.header.stamp = ros_sim_time;

	bot_pose.pose.position.x = tPosReads.Position.GetX();
	bot_pose.pose.position.y = tPosReads.Position.GetY();
	bot_pose.pose.position.z = tPosReads.Position.GetZ();

	bot_pose.pose.orientation.w = tPosReads.Orientation.GetW();
	bot_pose.pose.orientation.x = tPosReads.Orientation.GetX();
	bot_pose.pose.orientation.y = tPosReads.Orientation.GetY();
	bot_pose.pose.orientation.z = tPosReads.Orientation.GetZ();

	positionPublisher_->publish(bot_pose);

	/*********************************************
	 * Get readings from simple Radio
	 *********************************************/
	auto &sensIfs = const_cast<std::vector<argos::CCI_SimpleRadiosSensor::SInterface> &>(
		m_pcSRS->GetInterfaces());

	if (!sensIfs.empty() && !sensIfs[0].Messages.empty())
	{
		std_msgs::msg::Float64MultiArray msg;

		for (auto buf : sensIfs[0].Messages)
		{
			double val;
			buf >> val;

			msg.data.push_back(val);
		}

		radioDataPublisher_->publish(msg);
	}
	else
	{
		std_msgs::msg::Float64MultiArray msg;
		radioDataPublisher_->publish(msg);
	}

	/*********************************************
	 * Get readings from Range-And-Bearing-Sensor
	 *********************************************/
	const CCI_RangeAndBearingSensor::TReadings &tRabReads = m_pcRABS->GetReadings();
	if (!tRabReads.empty())
	{
		std_msgs::msg::Float64MultiArray rabSensorData;
		std::vector<geometry_msgs::msg::TransformStamped> transforms;
		for (size_t i = 0; i < tRabReads.size(); ++i)
		{
			double range = tRabReads[i].Range / 100; // cm->m
			double h_bearing = tRabReads[i].HorizontalBearing.GetValue();
			double v_bearing = tRabReads[i].VerticalBearing.GetValue();

			// 计算相对坐标
			double x = range * std::cos(v_bearing) * std::cos(h_bearing);
			double y = range * std::cos(v_bearing) * std::sin(h_bearing);
			double z = range * std::sin(v_bearing);

			// 尝试提取第一个 double 作为 ID
			double target_id = -1.0;
			CByteArray cBuf_copy = tRabReads[i].Data;

			bool first_value = true;
			double extractedValue;

			while (cBuf_copy.Size() >= sizeof(double))
			{
				cBuf_copy >> extractedValue;

				// 第一个值作为 ID，用于 child_frame 命名
				if (first_value)
				{
					target_id = extractedValue;
					first_value = false;
				}

				rabSensorData.data.push_back(extractedValue);
			}

			// 创建 TransformStamped
			geometry_msgs::msg::TransformStamped tf_msg;
			tf_msg.header.stamp = ros_sim_time; // argos_time

			std::stringstream parent_frame, child_frame;
			parent_frame << GetId() << "/base_link";
			child_frame << "bot" << static_cast<int>(target_id) << "/base_link";

			tf_msg.header.frame_id = parent_frame.str();
			tf_msg.child_frame_id = child_frame.str();

			tf_msg.transform.translation.x = x;
			tf_msg.transform.translation.y = y;
			tf_msg.transform.translation.z = z;

			// 没有方向信息就设为单位四元数
			tf_msg.transform.rotation.x = 0.0;
			tf_msg.transform.rotation.y = 0.0;
			tf_msg.transform.rotation.z = 0.0;
			tf_msg.transform.rotation.w = 1.0;

			transforms.push_back(tf_msg);
		}

		tf2_msgs::msg::TFMessage tf_msg;
		tf_msg.transforms = transforms;

		tfPublisher_->publish(tf_msg);

		rabDataPublisher_->publish(rabSensorData);
	}
	else
	{
		tf2_msgs::msg::TFMessage empty_tf_msg;
		tfPublisher_->publish(empty_tf_msg);

		std_msgs::msg::Float64MultiArray rabSensorData;
		rabDataPublisher_->publish(rabSensorData);
	}
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

	m_pcWheels->SetLinearVelocity(leftSpeed, rightSpeed);
	/*********************************************
	 * boardcast data via rab actuator
	 *********************************************/
	CByteArray cBuf;
	size_t maxBytes = m_pcRABA->GetSize();

	if (!pendingRabData.empty())
	{
		for (double v : pendingRabData)
		{
			cBuf << v;
		}
	}

	while (cBuf.Size() < maxBytes)
	{
		cBuf << static_cast<double>(-1.0);
	}

	m_pcRABA->SetData(cBuf);

	pendingRabData.clear();

	/*********************************************
	 * boardcast data via radio actuator
	 *********************************************/
	auto &radioActIfs = m_pcSRA->GetInterfaces();
	radioActIfs[0].Messages.clear();
	if (!pendingRadioData.data.empty())
	{
		for (double v : pendingRadioData.data)
		{
			CByteArray outBuf;
			outBuf << v;
			radioActIfs[0].Messages.push_back(outBuf);
		}
		pendingRadioData.data.clear();
	}
}

void ArgosRosFootbot::Reset()
{
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

void ArgosRosFootbot::rabActuatorCallback(const std_msgs::msg::Float64MultiArray &rabActuator)
{
	pendingRabData = rabActuator.data;
}

void ArgosRosFootbot::radioActuatorCallback(const std_msgs::msg::Float64MultiArray &radioActuator)
{
	pendingRadioData = radioActuator;
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
