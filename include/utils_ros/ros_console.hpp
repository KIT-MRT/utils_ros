#include <ros/ros.h>
#include <ros/console.h>

#include <boost/algorithm/string.hpp>

namespace utils_ros {

	/**
	 * Sets the logger level according to a standardize parameter
	 * name 'verbosity'.
	 *
	 * @param nodeHandle The ROS node handle to search for the
	 * parameter 'verbosity'.
	 */
	void setLoggerLevel (const ros::NodeHandle& nodeHandle);

	/**
	 * Show summary about node containing name, namespace,
	 * subscribed and advertised topics.
	 */
	void showNodeInfo ();
}
