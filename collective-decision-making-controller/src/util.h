/*
 * util.h
 *
 *  Created on: 05 Aug 2024
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
 *\brief Random seed. Default value (="-1") means time based.
 *
 * "Note that boost/c++11 random is slower (by one order of magnitude), but more precise,
 * than old classic rand()"
 *
 */
extern int gRandomSeed;

extern std::random_device rnd;
/**
 * \brief Returns an int with value drawn uniformly in [0,max)
 */
extern std::minstd_rand randint;
/**
 * \brief Engine
 */
extern std::mt19937 engine;
/**
 * \brief Uniform real distribution
 */
extern std::uniform_real_distribution<double> disRandom;
/**
 * \brief Normal distribution
 */
extern std::normal_distribution<> disNormal;
/**
 * \brief Iniform, unbiased
 */
extern std::uniform_int_distribution<int> randomSensor;
/**
 * Uniform in [0,1), return double
 */
#define random() disRandom(engine)
/**
 * \brief Normal distribution mean=0 and stddev=1 (use: mean+rand*stddev)
 */
#define randgaussian() disNormal(engine)

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

/*

 How to use Log?

 OPTION 1:
    with any new instance of roborobo, a gLogManager is created and available. You just have to use the two following commands:
    (1) gLogManager.write()
    (2) optionally: gLogManager.flush() -- note that this command is called automatically after each evaluation.

 OPTION 2:
    You can create as many new LogManager as you want. By default they will write in the same file as gLogManager. Here is how to change this:

        std::string filename = "logs/test.txt";
        std::ofstream file;

        file.open(filename.c_str());

        if(!file) {
            std::cout << "[error] Cannot open log file " << std::endl;
            exit (-1);
        }

        LogManager *lm = new LogManager();

        lm->setLogFile(file);

        lm->write("all work and no play makes Jack a dull boy ");

        lm->flush();

        file.close();

    Some remarks on option 2:
        (1) dont forget to reassign file target (cf. lm->setLogFile(...) )
        (2) dont forget to flush once in a while (no flush means information is not written on disk)
 */

/**
 * \brief Class to handle logging
 */
class Logger {

private:

	std::string buffer;
    std::ofstream *logFile;  // LogManager does not open/close. Assume it is handled elsewhere.

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
    static std::string gExperiementname;
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