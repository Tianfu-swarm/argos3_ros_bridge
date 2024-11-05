/*
 * util.h
 *
 *  Created on: 15 Aug 2024
 *  Author: Sindiso Mkhatshwa
 * 	Email: sindiso.mkhatshwa@uni-konstanz.de
 */

#ifndef UTIL_H_
#define UTIL_H_

/************************************************************************
 * INCLUDES
 ***********************************************************************/

#include <cstdlib> // RAND_MAX
#include <random>
#include <chrono>
#include <sys/time.h>
#include <fstream>
#include <iostream>
#include <boost/lexical_cast.hpp>
class Logger;

/**
 * \brief Log file
 */
extern std::ofstream gLogFile;

/**
 * \brief Return current time in a string, with readable format - e.g. 20100526-10h12m08s
 *
 * check: http://www.cplusplus.com/reference/clibrary/ctime/strftime/
 */
std::string getCurrentTimeAsReadableString();

/** \brief Return PID as a string. Useful to desambiguate timestamps */
std::string getpidAsReadableString();

/**
 * \brief Convert integer to string. check: http://notfaq.wordpress.com/2006/08/30/c-convert-int-to-string/
 * 	@param int value
 * 	@return std::string string representation
 */
std::string convertToString( int __value );

/**
 * \brief Class to handle logging
 */
class Logger {

private:

	std::string buffer;
    std::ofstream *logFile;  // LogManager does not open/close. Assume it is handled elsewhere.
    //std::ofstream gLogFile;

public:
    /**
     * \brief GLOBAL VARIABLES
     */

    /***************************************
     * Default logging
     **************************************/
    /**
     * \brief Log directory name
     */
    static std::string gLogDirectoryname;
    /**
     * \brief Experiment name
     */
    static std::string gExperimentname;
    /**
     * \brief Log file name
     */
    static std::string gLogFilename;
    /**
     * \brief Log full filename - directory
     */
    static std::string gLogFullFilename;
    
    /**
     * \brief Logger
     */
    static Logger *gLogger;
    /****************************************
     * Specified logging
     ***************************************/
    /**
     * \brief Robot state (commitment and opinion) log file
     */
    static std::ofstream gRobotStateLogFile;
    /**
     * \brief Robot state (commitment and opinion) logger
     */
    static Logger* gRobotStateLogger;
    /**
     * \brief Default Constructor
     */
    Logger();
    /**
     * \brief Constructor
     */
    Logger(std::ofstream __logFile);
    /**
     * \brief Destructor
     */
	virtual ~Logger();
	/**
	 * \brief Sets logging to defaul logger
	 */
    static Logger* make_DefaultLogger(); //(std::ofstream __logFile);
    /**
     * \brief Sets logger file
     */
    void setLoggerFile ( std::ofstream &__logFile );
    /**
     * \brief Writes to designated logfile
     */
	void write(std::string str);
	/**
	 * \brief Saves last written info
	 */
    void flush();
};

#endif /* UTIL_H_ */