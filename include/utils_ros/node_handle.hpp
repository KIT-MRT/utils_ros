#pragma once

#include <ros/node_handle.h>
#include <stdlib.h>

namespace utils_ros {

template<typename T>
inline void getParam(const ros::NodeHandle& node_handle, const std::string key,
		T& val) {

	if (!node_handle.getParam(key, val)) {
		ROS_ERROR_STREAM("Undefined parameter '" << key <<"'.");
		exit(EXIT_FAILURE);
	}
}
}
