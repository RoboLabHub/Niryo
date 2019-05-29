#ifndef _ROS_H_
#define _ROS_H_

#include "ros/node_handle.h"
#include "ArduinoHardware.h"

namespace ros
{
  /* Publishers, Subscribers, Buffer Sizes for OpenCR*/
  typedef NodeHandle_<ArduinoHardware, 25, 25, 1024, 1024> NodeHandle;
}

#endif
