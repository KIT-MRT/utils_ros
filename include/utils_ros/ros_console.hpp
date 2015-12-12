#include <ros/ros.h>
#include <boost/algorithm/string.hpp>

#include <generic_logger/generic_logger.hpp>
#include <generic_logger/sinks/ros_sink.h>

namespace utils_ros {

/**
 * Sets the logger level according to a standardize parameter
 * name 'verbosity'.
 *
 * @param nodeHandle The ROS node handle to search for the
 * parameter 'verbosity'.
 */
inline void setLoggerLevel(const ros::NodeHandle& nodeHandle) {

	using namespace generic_logger;
	using namespace ros::console;

	set_name(ros::this_node::getName());

	std::string verbosity;
	if (nodeHandle.getParam("verbosity", verbosity)) {

		/**
		 * Generic logger level
		 */
		boost::to_lower(verbosity);
		set_sink(std::make_shared<generic_logger::sinks::ros_sink>());
		set_level(verbosity);

		/**
		 * Set ROS console level
		 */
		ros::console::Level level_ros;
		bool valid_verbosity {true};
		if (verbosity == "debug") {
			level_ros = ros::console::levels::Debug;
		} else if (verbosity == "info") {
			level_ros = ros::console::levels::Info;
		} else if (verbosity == "warn") {
			level_ros = ros::console::levels::Warn;
		} else if (verbosity == "error") {
			level_ros == ros::console::levels::Error;
		} else if (verbosity == "fatal") {
			level_ros == ros::console::levels::Fatal;
		} else {
			ROS_WARN_STREAM(
					"Invalid verbosity level specified: " << verbosity << "! Falling back to INFO.");
			valid_verbosity = false;
		}
		if (valid_verbosity) {
			if (ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, level_ros)) {
				ros::console::notifyLoggerLevelsChanged();
				ROS_DEBUG_STREAM("Verbosity set to " << verbosity);
			}
		}

	} else {
		ROS_ERROR_STREAM("Undefined parameter 'verbosity'.");
		ros::shutdown();
	}
}

/**
 * Show summary about node containing name, namespace,
 * subscribed and advertised topics.
 */
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
			"Started '" << getName() << "' in namespace '" << getNamespace() << "'." << std::endl << "Subscribed topics: " << std::endl << msg_subscr.str() << "Advertised topics: " << std::endl << msg_advert.str());
}
}
