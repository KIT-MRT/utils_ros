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

	void showNodeInfo() {

		using namespace ros::this_node;

		std::vector<std::string> subscribed_topics, advertised_topics;
		getSubscribedTopics(subscribed_topics);
		getAdvertisedTopics(advertised_topics);

		std::ostringstream msg_subscr, msg_advert;
		for (auto const& t : subscribed_topics) {
			msg_subscr << t << std::endl;
		}
		for (auto const& t : advertised_topics) {
			msg_advert << t << std::endl;
		}

		ROS_INFO_STREAM(
				"Started '" << getName() <<
				"' in namespace '" << getNamespace() << "'." << std::endl <<
				"Subscribed topics: " << std::endl << msg_subscr.str() <<
				"Advertised topics: " << std::endl << msg_advert.str());
	}

}
