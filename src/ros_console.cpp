#include "ros_console.hpp"

namespace utils_ros {

	void setLoggerLevel (const ros::NodeHandle& nodeHandle) {
		std::string verbosity;
		if (nodeHandle.getParam("verbosity", verbosity)) {
			ros::console::Level selectedLevel = ros::console::levels::Info;
			boost::to_lower(verbosity);
			bool validVerbosity = true;
			if (verbosity == "debug") {
				selectedLevel = ros::console::levels::Debug;
			}
			else if (verbosity == "info") {
				selectedLevel = ros::console::levels::Info;
			}
			else if (verbosity == "warn") {
				selectedLevel = ros::console::levels::Warn;
			}
			else if (verbosity == "error") {
				selectedLevel == ros::console::levels::Error;
			}
			else if (verbosity == "fatal") {
				selectedLevel == ros::console::levels::Fatal;
			}
			else {
				ROS_WARN_STREAM("Invalid verbosity level specified: " << verbosity << "! Falling back to INFO.");
				validVerbosity = false;
			}
			if (validVerbosity) {
				if (ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, selectedLevel)) {
					ros::console::notifyLoggerLevelsChanged();
					ROS_INFO_STREAM("Verbosity is set to: " << verbosity);
				}
			}
		}
	}
}
