#pragma once
#include "transform_lib/eigen_transform_conversions.h"
#include "geometry_msgs/Quaternion.h"

namespace transform_lib
{
    geometry_msgs::Quaternion yawToQuaternion(float yaw, bool isInDegrees);

    void yawToQuaternion(float yaw, bool isInDegrees, geometry_msgs::Quaternion &result);
}