#include "transform_lib/quaternion_conversions.h"

geometry_msgs::Quaternion transform_lib::yawToQuaternion(float yaw, bool isInDegrees)
{
    geometry_msgs::Quaternion result;
    if(isInDegrees)
    {
        yaw *= M_PI / 180;
    }
    result.w = cos(yaw/2) - 0;
    result.x = 0;
    result.y = 0;
    result.z = sin(yaw/2) - 0;
    return result;
}

void transform_lib::yawToQuaternion(float yaw, bool isInDegrees, geometry_msgs::Quaternion &result)
{
    if(isInDegrees)
    {
        yaw *= M_PI / 180;
    }
    result.w = cos(yaw/2) - 0;
    result.x = 0;
    result.y = 0;
    result.z = sin(yaw/2) - 0;
}