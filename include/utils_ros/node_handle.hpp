#pragma once

#include <stdlib.h>
#include <ros/node_handle.h>

namespace utils_ros {

template <typename T>
inline void getParam(const ros::NodeHandle& nodeHandle, const std::string& key, T& val) {
    if (!nodeHandle.getParam(key, val)) {
        ROS_ERROR_STREAM("Parameter '" << key << "' is not defined.");
        std::exit(EXIT_FAILURE);
    }
}

template <typename T>
inline void getParam(const ros::NodeHandle& nodeHandle, const std::string& key, T& val, const T& defaultValue) {
    nodeHandle.param(key, val, defaultValue);
}

template <typename T>
inline T getParam(const ros::NodeHandle& nodeHandle, const std::string& key) {
    if (!nodeHandle.hasParam(key)) {
        ROS_ERROR_STREAM("Parameter '" << key << "' is not defined.");
        std::exit(EXIT_FAILURE);
    }
    T val;
    nodeHandle.getParam(key, val);
    return val;
}

template <typename T>
inline T getParam(const ros::NodeHandle& nodeHandle, const std::string& key, const T& defaultValue) {
    if (!nodeHandle.hasParam(key)) {
        return defaultValue;
    }
    T val;
    nodeHandle.getParam(key, val);
    return val;
}
}
