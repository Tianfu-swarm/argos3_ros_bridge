#include "trajectory_loop_functions.h"

std::ofstream gLogFile;

/****************************************/
/****************************************/

/**
 * Initialize the node before creating the publishers
 * and subscribers. Otherwise we get a guard-error during
 * compilation if we initialize the node after.
 */
std::shared_ptr<rclcpp::Node> initNode() {
  int argc = 1;
  char *argv = (char *) "";
  
  //block until context is true
  if (rclcpp::get_contexts().empty()){rclcpp::init(argc, &argv);}
  return std::make_shared<rclcpp::Node>("argos_ros_loop_funs_node");

}
std::shared_ptr<rclcpp::Node> CTrajectoryLoopFunctions::nodeHandle = initNode();

/*
 * To reduce the number of waypoints stored in memory,
 * consider two robot positions distinct if they are
 * at least MIN_DISTANCE away from each other
 * This constant is expressed in meters
 */
static const Real MIN_DISTANCE = 0.05f;
/* Convenience constant to avoid calculating the square root in PostStep() */
static const Real MIN_DISTANCE_SQUARED = MIN_DISTANCE * MIN_DISTANCE;

/****************************************/
/****************************************/


void CTrajectoryLoopFunctions::Init(TConfigurationNode& t_tree) {
   /********************************
	 * Create the topics to publish
	 *******************************/
   signal_.signal = 0;
	std::stringstream killTopic;
	killTopic 		<< "/kill";
	killPublisher_ = CTrajectoryLoopFunctions::nodeHandle -> create_publisher<collective_decision_making::msg::Signal>(killTopic.str(), 1);
	
   gStartTime_ = getCurrentTimeAsReadableString();
   steps_ = 0;
   try {
      TConfigurationNode& tLogging = GetNode(t_tree, "logging");
      
      /* Get the logs directory from XML */
      GetNodeAttribute(tLogging, "directory", Logger::gLogDirectoryname);
      /* Get the experiment name from XML */
      GetNodeAttribute(tLogging, "experiment_name", Logger::gExperimentname);
      /* Get the frequency of logging from XML */
      GetNodeAttribute(tLogging, "frequency", logFreq_);
      /* Get the light radius from which to terminate simulation from XML */
      GetNodeAttribute(tLogging, "light_radius", lightRadius_);
      initLogging();
   }
   catch(CARGoSException& ex) {
      THROW_ARGOSEXCEPTION_NESTED("Error parsing loop functions!", ex);
   }
   /*
    * Go through all the robots in the environment
    * and create an entry in the waypoint map for each of them
    */
   /* Get the map of all foot-bots from the space */
   CSpace::TMapPerType& tFBMap = GetSpace().GetEntitiesByType("foot-bot");
   /* Go through them */
   for(CSpace::TMapPerType::iterator it = tFBMap.begin();
       it != tFBMap.end();
       ++it) {
      /* Create a pointer to the current foot-bot */
      CFootBotEntity* pcFB = any_cast<CFootBotEntity*>(it->second);
      /* Create a waypoint vector */
      m_tWaypoints[pcFB] = std::vector<CVector3>();
      /* Add the initial position of the foot-bot */
      m_tWaypoints[pcFB].push_back(pcFB->GetEmbodiedEntity().GetOriginAnchor().Position);
   }

   
}

void CTrajectoryLoopFunctions::initLogging(){
	// ==== create specific "maps" logger file
	Logger:: gLogFilename =	Logger::gExperimentname + "_" + "positions_" + gStartTime_ + "_" + getpidAsReadableString() + ".csv";
	Logger::gLogFullFilename = Logger::gLogDirectoryname + "/" + Logger::gLogFilename;
	Logger::gRobotStateLogFile.open(Logger::gLogFullFilename.c_str());

	if(!Logger::gRobotStateLogFile) {
		std::cout << "[CRITICAL] Cannot open \"robot state\" log file " << Logger::gLogFullFilename << "." << std::endl;
		exit(-1);
	}
   std::cout << "Loop function - Log filename: " << Logger::gLogFullFilename << std::endl;
	Logger::gRobotStateLogger = new Logger();
	Logger::gRobotStateLogger->setLoggerFile(Logger::gRobotStateLogFile);
	Logger::gRobotStateLogger->write("Time, ID, x, y");
	Logger::gRobotStateLogger->write(std::string("\n"));
	Logger::gRobotStateLogger->flush();
}

/****************************************/
/****************************************/

void CTrajectoryLoopFunctions::Reset() {
   /*
    * Clear all the waypoint vectors
    */
   /* Get the map of all foot-bots from the space */
   CSpace::TMapPerType& tFBMap = GetSpace().GetEntitiesByType("foot-bot");
   /* Go through them */
   for(CSpace::TMapPerType::iterator it = tFBMap.begin();
       it != tFBMap.end();
       ++it) {
      /* Create a pointer to the current foot-bot */
      CFootBotEntity* pcFB = any_cast<CFootBotEntity*>(it->second);
      /* Clear the waypoint vector */
      m_tWaypoints[pcFB].clear();
      /* Add the initial position of the foot-bot */
      m_tWaypoints[pcFB].push_back(pcFB->GetEmbodiedEntity().GetOriginAnchor().Position);
   }
}

/****************************************/
/****************************************/

void CTrajectoryLoopFunctions::PostStep() {
   /* Get the map of all foot-bots from the space */
   CSpace::TMapPerType& tFBMap = GetSpace().GetEntitiesByType("foot-bot");
   /* Get the map of all light-sources from the space */
   CSpace::TMapPerType& tLSMap = GetSpace().GetEntitiesByType("light");
   float xSummation = 0.0f;
   float ySummation = 0.0f;
   unsigned int allBots = 0;
   /* Go through all the foot-bots */
   for(CSpace::TMapPerType::iterator it = tFBMap.begin();
       it != tFBMap.end();
       ++it) {
      /* Create a pointer to the current foot-bot */
      CFootBotEntity* pcFB = any_cast<CFootBotEntity*>(it->second);
      /* Add the current position of the foot-bot if it's sufficiently far from the last */
      if(SquareDistance(pcFB->GetEmbodiedEntity().GetOriginAnchor().Position,
                        m_tWaypoints[pcFB].back()) > MIN_DISTANCE_SQUARED) {
         m_tWaypoints[pcFB].push_back(pcFB->GetEmbodiedEntity().GetOriginAnchor().Position);
      }
      if (steps_ % logFreq_ == 0){
         std::stringstream ss;
         ss    << steps_ << ", " 
               << pcFB->GetId() << ", " 
               << pcFB->GetEmbodiedEntity().GetOriginAnchor().Position.GetX() << ", " 
               << pcFB->GetEmbodiedEntity().GetOriginAnchor().Position.GetY() <<std::endl;
         Logger::gRobotStateLogger->write(ss.str());
         Logger::gRobotStateLogger->flush();
         // Add all foot-bot positions to be able to compute "center-of-mass"
         xSummation += pcFB->GetEmbodiedEntity().GetOriginAnchor().Position.GetX();
         ySummation += pcFB->GetEmbodiedEntity().GetOriginAnchor().Position.GetY();
         allBots ++;
      }
   }
   if (steps_ % logFreq_ == 0){
      // compute the 'center-of-mass' of the swarm
      float xAverage = xSummation / allBots;
      float yAverage = ySummation / allBots;
      CVector3 posAverage(xAverage, yAverage, 0);
      /* Go through light-sources in the environment */
      for(CSpace::TMapPerType::iterator it = tLSMap.begin();
         it != tLSMap.end();
         ++it) {
         /* Create a pointer to the current light-source */
         CLightEntity* pcLS = any_cast<CLightEntity*>(it->second); 
         CVector3 posLS(pcLS->GetInitPosition().GetX(), pcLS->GetInitPosition().GetY(), 0);  
         if(SquareDistance(posLS,
                           posAverage) < lightRadius_) {
            std::cout << "Simulation done." << std::endl;
            GetSimulator().Terminate();
            
            signal_.signal = 1;
            std::cout << "sending kill signal" << std::endl;
            killPublisher_ -> publish(signal_);
            rclcpp::shutdown();
            exit(0);
         }
      }
      killPublisher_ -> publish(signal_);
   }
   steps_ ++;
}

/****************************************/
/****************************************/

REGISTER_LOOP_FUNCTIONS(CTrajectoryLoopFunctions, "trajectory_loop_functions")
