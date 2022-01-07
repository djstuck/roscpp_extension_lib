#include "transform_lib/nan_check.h"

bool transform_lib::isPoseNAN(geometry_msgs::Pose& p)
{
    if(isnanf(p.position.x) || isnanf(p.position.y) || isnanf(p.position.z) || isnanf(p.orientation.x) || isnanf(p.orientation.y) || isnanf(p.orientation.z) || isnanf(p.orientation.w))
        return true;
    else 
        return false;
}

bool transform_lib::isPoseNAN(geometry_msgs::PoseStamped& p)
{
    if(isnanf(p.pose.position.x) || isnanf(p.pose.position.y) || isnanf(p.pose.position.z) || isnanf(p.pose.orientation.x) || isnanf(p.pose.orientation.y) || isnanf(p.pose.orientation.z) || isnanf(p.pose.orientation.w))
        return true;
    else 
        return false;
}