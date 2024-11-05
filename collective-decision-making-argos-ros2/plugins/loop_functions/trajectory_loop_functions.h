#ifndef TRAJECTORY_LOOP_FUNCTIONS_H
#define TRAJECTORY_LOOP_FUNCTIONS_H

#include <argos3/core/simulator/loop_functions.h>
#include <argos3/plugins/robots/foot-bot/simulator/footbot_entity.h>
#include <argos3/plugins/simulator/entities/light_entity.h>
#include "util.h"
#include "plugins/argos_ros2_foot_bot/argos_ros_footbot.h"

/**
 * ROS2 Imports
 */
#include "rclcpp/rclcpp.hpp"
#include "collective_decision_making/msg/signal.hpp"

using namespace argos;

class CTrajectoryLoopFunctions : public CLoopFunctions {

public:

   typedef std::map<CFootBotEntity*, std::vector<CVector3> > TWaypointMap;
   TWaypointMap m_tWaypoints;
   
public:

   virtual ~CTrajectoryLoopFunctions() {}

   virtual void Init(TConfigurationNode& t_tree);

   void initLogging();

   virtual void Reset();

   virtual void PostStep();

   inline const TWaypointMap& GetWaypoints() const {
      return m_tWaypoints;
   }

   static std::shared_ptr<rclcpp::Node> nodeHandle;

private:
   int steps_; 
   std::string logDir_;
   int logFreq_;
   float lightRadius_;
   std::string gStartTime_;
   collective_decision_making::msg::Signal signal_;
   rclcpp::Publisher<collective_decision_making::msg::Signal>::SharedPtr killPublisher_;
		

};

#endif
