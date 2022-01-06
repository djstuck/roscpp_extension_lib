#pragma once
#include "transform_lib/eigen_transform_conversions.h"
#include "geometry_msgs/Quaternion.h"

namespace transform_lib
{
    geometry_msgs::Quaternion yawToQuaternion(float yaw, bool isInDegrees);

    void yawToQuaternion(float yaw, bool isInDegrees, geometry_msgs::Quaternion &result);

    float vectorToAngle(float& x, float& y);
    void vectorToAngle(float& x, float& y, float &angle);

    inline geometry_msgs::Quaternion vectorToQuaternion(float& x, float& y)
    {
        return yawToQuaternion(vectorToAngle(x, y), false);
    }
    
    inline void vectorToQuaternion(float& x, float& y, geometry_msgs::Quaternion& result)
    {
        yawToQuaternion(vectorToAngle(x, y), false, result);
    }

    void rotateQuaternionByAngle(float xx, float yy, float zz, float& angle, geometry_msgs::Quaternion& result);

    inline void rotateQuaternionByYaw(float yaw, bool isInDegrees, geometry_msgs::Quaternion& result)
    {
        if(isInDegrees)
            yaw *= M_PI / 180;
        rotateQuaternionByAngle(0, 0, 1, yaw, result);
    }

    void normalize(geometry_msgs::Quaternion& result);

    inline float dotProduct(geometry_msgs::Quaternion& q)
    {
        return (q.x * q.x) + (q.y * q.y) + (q.z * q.z) + (q.w * q.w);
    }

    void multiplyQuaternions(geometry_msgs::Quaternion& q1, geometry_msgs::Quaternion& q2, geometry_msgs::Quaternion& result);

    inline void multiplyQuaternions(geometry_msgs::Quaternion& result, geometry_msgs::Quaternion& q2)
    {
        geometry_msgs::Quaternion q;
        multiplyQuaternions(result, q2, q);
        result = q;
    }


}