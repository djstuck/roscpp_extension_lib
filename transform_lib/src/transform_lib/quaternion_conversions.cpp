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

float transform_lib::vectorToAngle(float& x, float& y)
{
    float angle;
    if(x > 0.0 && y > 0.0)
        angle = atan2(y, x);
    else if(x < 0.0)  //if y is positive or negative
        angle = M_PI + atan2(y, x);
    else
        angle = M_PI * 2 + atan2(y, x);

    return angle;
}

void transform_lib::vectorToAngle(float& x, float& y, float &angle)
{
    if(x > 0.0 && y > 0.0)
        angle = atan2(y, x);
    else if(x < 0.0)  //if y is positive or negative
        angle = M_PI + atan2(y, x);
    else
        angle = M_PI * 2 + atan2(y, x);
}

void transform_lib::rotateQuaternionByAngle(float xx, float yy, float zz, float& angle, geometry_msgs::Quaternion& result)
{
    geometry_msgs::Quaternion q2;
    float factor = sinf(angle / 2);
    q2.x = xx * factor;
    q2.y = yy * factor;
    q2.z = zz * factor;
    q2.w = cosf(angle / 2);
    normalize(q2); 
    multiplyQuaternions(result, q2);  
}

void transform_lib::normalize(geometry_msgs::Quaternion& result)
{
    float inv = 1 / sqrtf(dotProduct(result));
    result.x *=inv;
    result.y *=inv;
    result.z *=inv;
    result.w *=inv;
}

void transform_lib::multiplyQuaternions(geometry_msgs::Quaternion& q1, geometry_msgs::Quaternion& q2, geometry_msgs::Quaternion& result)
{
    result.x =  q1.x * q2.w + q1.y * q2.z - q1.z * q2.y + q1.w * q2.x;
    result.y = -q1.x * q2.z + q1.y * q2.w + q1.z * q2.x + q1.w * q2.y;
    result.z =  q1.x * q2.y - q1.y * q2.x + q1.z * q2.w + q1.w * q2.z;
    result.w = -q1.x * q2.x - q1.y * q2.y - q1.z * q2.z + q1.w * q2.w;
}