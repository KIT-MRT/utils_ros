#pragma once

#include <ros/node_handle.h>
#include <stdlib.h>

namespace utils_ros {

template<typename T>
inline void getParam(const ros::NodeHandle& nodeHandle, const std::string key, T& val) {
	if (!nodeHandle.getParam(key, val)) {
		ROS_ERROR_STREAM("Parameter '" << key << "' is not defined.");
		std::exit(EXIT_FAILURE);
	}
}

template<typename T>
inline void getParam(const ros::NodeHandle& nodeHandle, const std::string key, T& val, const T& defaultValue) {
    nodeHandle.param(key, val, defaultValue);
}
}
