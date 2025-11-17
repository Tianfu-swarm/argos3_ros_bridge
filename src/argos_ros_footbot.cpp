/* Include the controller definition */
#include "argos_ros_footbot.h"
using namespace std;
using namespace geometry_msgs::msg;
using std::placeholders::_1;

ArgosRosFootbot::ArgosRosFootbot() : m_pcWheels(NULL),
									 //  m_pcLight(NULL),
									 m_pcProximity(NULL),
									 m_pcPosition(NULL),
									 m_pcRABA(NULL),
									 m_pcRABS(NULL),
									 m_pcBaseGround(NULL),
									 m_pcMotorGround(NULL),
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
	int argc = 0;
	char **argv = nullptr;
	if (!rclcpp::ok())
	{
		rclcpp::init(argc, argv);
	}

	UInt32 num_threads = argos::CSimulator::GetInstance().GetNumThreads();

	std::string node_name = GetId() + "_argos3_ros_bridge";
	nodeHandle_ = std::make_shared<rclcpp::Node>(node_name);

	/********************************
	 * Create the topics to publish
	 *******************************/
	stringstream lightTopic, blobTopic, proxTopic, positionTopic, rabDataTopic, tfTopic, radioTopic, groundTopic, motorGroundTopic;
	// lightTopic << "/" << GetId() << "/light";
	blobTopic << "/" << GetId() << "/blob";
	proxTopic << "/" << GetId() << "/proximity_point";
	positionTopic << "/" << GetId() << "/pose";
	rabDataTopic << "/" << GetId() << "/rab_sensor";
	tfTopic << "/" << GetId() << "/rab_tf";
	radioTopic << "/" << GetId() << "/radio_sensor";
	groundTopic << "/" << GetId() << "/base_ground_sensor";
	motorGroundTopic << "/" << GetId() << "/motor_ground_sensor";

	promixityPublisher_ = nodeHandle_->create_publisher<sensor_msgs::msg::PointCloud2>(proxTopic.str(), 10);
	positionPublisher_ = nodeHandle_->create_publisher<geometry_msgs::msg::PoseStamped>(positionTopic.str(), 10);
	rabDataPublisher_ = nodeHandle_->create_publisher<std_msgs::msg::Float64MultiArray>(rabDataTopic.str(), 10);
	tfPublisher_ = nodeHandle_->create_publisher<tf2_msgs::msg::TFMessage>(tfTopic.str(), 10);
	radioDataPublisher_ = nodeHandle_->create_publisher<std_msgs::msg::Float64MultiArray>(radioTopic.str(), 10);
	baseGroundPublisher_ = nodeHandle_->create_publisher<std_msgs::msg::Float64MultiArray>(groundTopic.str(), 10);
	motorGroundPublisher_ = nodeHandle_->create_publisher<std_msgs::msg::Float64MultiArray>(motorGroundTopic.str(), 10);

	/*********************************
	 * Create subscribers
	 ********************************/
	stringstream cmdVelTopic,
		rabActuatorTopic, radioActuatorTopic;
	cmdVelTopic << "/" << GetId() << "/cmd_vel";
	rabActuatorTopic << "/" << GetId() << "/rab_actuator";
	radioActuatorTopic << "/" << GetId() << "/radio_actuator";

	cmdVelSubscriber_ = nodeHandle_->create_subscription<Twist>(cmdVelTopic.str(), 10, std::bind(&ArgosRosFootbot::cmdVelCallback, this, _1));
	rabActuatorSubscriber_ = nodeHandle_->create_subscription<std_msgs::msg::Float64MultiArray>(rabActuatorTopic.str(), 10, std::bind(&ArgosRosFootbot::rabActuatorCallback, this, _1));
	radioActuatorSubscriber_ = nodeHandle_->create_subscription<std_msgs::msg::Float64MultiArray>(radioActuatorTopic.str(), 10, std::bind(&ArgosRosFootbot::radioActuatorCallback, this, _1));
	/********************************
	 * Get sensor/actuator handles
	 ********************************/
	// m_pcLight = GetSensor<CCI_FootBotLightSensor>("footbot_light");
	m_pcProximity = GetSensor<CCI_FootBotProximitySensor>("footbot_proximity");
	m_pcPosition = GetSensor<CCI_PositioningSensor>("positioning");
	m_pcRABS = GetSensor<CCI_RangeAndBearingSensor>("range_and_bearing");
	m_pcSRS = GetSensor<CCI_SimpleRadiosSensor>("simple_radios");
	m_pcBaseGround = GetSensor<CCI_FootBotBaseGroundSensor>("footbot_base_ground");
	m_pcMotorGround = GetSensor<CCI_FootBotMotorGroundSensor>("footbot_motor_ground");

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

	// rclcpp::spin_some(ArgosRosFootbot::nodeHandle);
	rclcpp::spin_some(nodeHandle_);

	/***********************************
	 * Get readings from base ground sensor
	 ***********************************/
	// 取 ground sensor 数值
	const auto &r = m_pcBaseGround->GetReadings(); // TReadings, size=8（见头文件注释顺序）

	// 填充 ROS2 消息
	std_msgs::msg::Float64MultiArray ground_msg;
	ground_msg.data.reserve(r.size());
	for (const auto &s : r)
	{
		ground_msg.data.push_back(static_cast<double>(s.Value));
	}

	// 发布
	baseGroundPublisher_->publish(ground_msg);

	/***********************************
	 * Get readings from motor ground sensor
	 ***********************************/
	if (m_pcMotorGround)
	{
		const auto &mg = m_pcMotorGround->GetReadings(); // TReadings, size=4
		std_msgs::msg::Float64MultiArray msg;
		msg.data.reserve(mg.size());
		for (const auto &s : mg)
			msg.data.push_back(static_cast<double>(s.Value));
		motorGroundPublisher_->publish(msg);
	}

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

	// 衰减系数 λ
	const double Rmax = 0.10; // footbot 近红外量程 10 cm

	for (size_t i = 0; i < tProxReads.size(); ++i)
	{

		if (tProxReads[i].Value == 0)
		{
			continue;
		}

		double range = (1.0 - tProxReads[i].Value) * (Rmax / 0.9); // 计算实际距离
		double angle = tProxReads[i].Angle.GetValue();			   // 获取角度（单位：弧度）

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

	/*********************************************
	 * Get readings from simple Radio
	 *********************************************/

	auto &sensIfs = const_cast<std::vector<argos::CCI_SimpleRadiosSensor::SInterface> &>(m_pcSRS->GetInterfaces());
	std_msgs::msg::Float64MultiArray msg;
	if (!sensIfs.empty() && !sensIfs[0].Messages.empty())
	{
		for (auto buf : sensIfs[0].Messages)
		{
			double val;
			buf >> val;

			msg.data.push_back(val);
		}
		sensIfs[0].Messages.clear();
	}

	radioDataPublisher_->publish(msg);

	// std::cout << "[argos_bridge_" << GetId() << " ][" << ros_sim_time.seconds() << "] Received from sensor and send to ros: [";
	// for (size_t i = 0; i < msg.data.size(); ++i)
	// {
	// 	std::cout << msg.data[i];
	// 	if (i != msg.data.size() - 1)
	// 		std::cout << ", ";
	// }
	// std::cout << "]" << std::endl;

	/*********************************************
	 * Get readings from Range-And-Bearing-Sensor
	 *********************************************/
	const CCI_RangeAndBearingSensor::TReadings &tRabReads = m_pcRABS->GetReadings();
	if (!tRabReads.empty())
	{
		std_msgs::msg::Float64MultiArray rabSensorData;
		rabSensorData.data.clear();
		rabSensorData.data.reserve(tRabReads.size() * 8); // 预留，避免频繁扩容

		std::vector<geometry_msgs::msg::TransformStamped> transforms;
		transforms.clear();
		transforms.reserve(tRabReads.size());

		const std::string parent_frame = GetId() + std::string("/base_link");

		// 可选：限制最多发布的邻居 TF 数，减轻高密度压力
		constexpr size_t MAX_TF_NEIGHBORS = 64;

		for (size_t i = 0; i < tRabReads.size(); ++i)
		{
			double range = tRabReads[i].Range / 100.0; // cm->m
			double h_bearing = tRabReads[i].HorizontalBearing.GetValue();
			double v_bearing = tRabReads[i].VerticalBearing.GetValue();

			const double ch = std::cos(h_bearing);
			const double sh = std::sin(h_bearing);
			const double cv = std::cos(v_bearing);
			const double sv = std::sin(v_bearing);

			const double x = range * cv * ch;
			const double y = range * cv * sh;
			const double z = range * sv;

			// 只提取“有效载荷”，遇到 padding（<0）立刻停止
			double target_id = -1.0;
			CByteArray buf = tRabReads[i].Data; // 拷贝一份再读，避免破坏原数据
			bool first = true;

			while (buf.Size() >= sizeof(double))
			{
				double v;
				buf >> v;

				if (v < 0.0)
					break; // ← 哨兵：到填充值就停止读取（不把 padding 推给 ROS）

				if (first)
				{
					target_id = v;
					first = false;
				}
				rabSensorData.data.push_back(v); // 只推有效载荷
			}

			if (target_id < 0.0)
			{
				continue; // 无有效 ID，跳过 TF
			}

			geometry_msgs::msg::TransformStamped tf_msg;
			tf_msg.header.stamp = ros_sim_time;
			tf_msg.header.frame_id = parent_frame;
			tf_msg.child_frame_id = "bot" + std::to_string(static_cast<int>(target_id)) + "/base_link";

			tf_msg.transform.translation.x = x;
			tf_msg.transform.translation.y = y;
			tf_msg.transform.translation.z = z;

			tf_msg.transform.rotation.x = 0.0;
			tf_msg.transform.rotation.y = 0.0;
			tf_msg.transform.rotation.z = 0.0;
			tf_msg.transform.rotation.w = 1.0;

			transforms.push_back(std::move(tf_msg));
			if (transforms.size() >= MAX_TF_NEIGHBORS)
				break; // 可选：限制 TF 数
		}

		tf2_msgs::msg::TFMessage tf_msg;
		tf_msg.transforms = std::move(transforms);

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
}

void ArgosRosFootbot::Reset()
{
}

void ArgosRosFootbot::cmdVelCallback(const Twist &twist)
{
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
