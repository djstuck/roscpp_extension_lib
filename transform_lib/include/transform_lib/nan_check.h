#pragma once

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Pose.h>


namespace transform_lib{

    bool isPoseNAN(geometry_msgs::Pose& p);

    bool isPoseNAN(geometry_msgs::PoseStamped& p);

} // namespace transform_lib