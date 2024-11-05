/*
 * util.cpp
 *
 *  Created on: 15 Aug 2024
 *  Author: Sindiso Mkhatshwa
 * 	Email: sindiso.mkhatshwa@uni-konstanz.de
 */
#include "util.h"

//std::ofstream gLogFile;

std::ofstream Logger::gRobotStateLogFile;
Logger* Logger::gRobotStateLogger = NULL;
std::string Logger::gLogDirectoryname;
std::string Logger::gExperimentname;
std::string Logger::gLogFilename;
std::string Logger::gLogFullFilename;



std::string getCurrentTimeAsReadableString(){
	// --- get current time information

	time_t now = time(NULL);
	char timestamp[19] = "";
	//strftime (timestamp, 19,"%Y%m%d-%Hh%Mm%Ss", localtime(&now));
	strftime (timestamp, 19,"%Y%m%d-%Hh%Mm%Ss", localtime(&now));
	std::string s = timestamp;

	// --- get milliseconds resolution (note: could be used to replace code block above - left for tutorial)

	struct timeval now2;
    int mtime;
    gettimeofday(&now2, NULL);

	mtime = now2.tv_usec;
	if ( mtime < 100000 )
	{
		s+="0";
		if ( mtime < 10000 )
			s+="0";
		if ( mtime < 1000 )
			s+="0";
		if ( mtime < 100 )
			s+="0";
		if ( mtime < 10 )
			s+="0";
	}
	s += convertToString(mtime) + "us"; // microseconds

	return s;

}
std::string getpidAsReadableString(){
    return boost::lexical_cast<std::string>((long)::getpid());
}
std::string convertToString( int __value ){
	std::string s;
	std::stringstream sOutTmp;
	sOutTmp << __value;
	s = sOutTmp.str();

	return s;
}
/*****************************************************
 * Logger class definition
 ****************************************************/
Logger::Logger(std::ofstream __logFile){
    logFile = &__logFile;
	buffer = "";
}

Logger::Logger(){
    logFile = &gLogFile;
}

Logger::~Logger(){}

void Logger::write(std::string str){
    buffer += str;
}

void Logger::flush(){
    if ( ! buffer.empty() )
    {
        (*logFile) << buffer;
        buffer.clear();
    }
}

Logger* Logger::make_DefaultLogger(){
    return new Logger();
}

void Logger::setLoggerFile ( std::ofstream &__logFile ){
    logFile = &__logFile;
}
