#include "transform_lib/transform_functions.h"

void transform_lib::getTransform(std::string &target_frame, std::string &source_frame, tf2_ros::Buffer* tfBuffer, geometry_msgs::TransformStamped &transform_result, double& time_offset, ros::Duration &duration, std::string &error_msg)
{
    bool transform_exception;
    do
    {
        transform_exception = false;
        try
        {
            transform_result = tfBuffer -> lookupTransform(target_frame, source_frame, ros::Time(time_offset), duration);
        }
        catch(tf2::TransformException &ex) 
        {
            ROS_WARN("Failure %s\n", ex.what()); //Print exception which was caught
            ROS_WARN("%s", error_msg.c_str()); //Print extended error message
            ros::WallDuration(1.0).sleep();
            transform_exception = true;
        }
        
    } while (transform_exception);
    
}

int transform_lib::getTransformTimed(std::string &target_frame, std::string &source_frame, tf2_ros::Buffer* tfBuffer, geometry_msgs::TransformStamped &transform_result, double& time_offset, ros::Duration &duration, std::string &error_msg)
{
    int64_t time_span = -1000;
    bool transform_exception;
    do
    {

        transform_exception = false;
        try
        {
            std::chrono::high_resolution_clock::time_point t1 = std::chrono::high_resolution_clock::now();

            transform_result = tfBuffer -> lookupTransform(target_frame, source_frame, ros::Time(time_offset), duration);

            std::chrono::high_resolution_clock::time_point t2 = std::chrono::high_resolution_clock::now();
            time_span = std::chrono::duration_cast<std::chrono::microseconds>(t2 - t1).count();
        }
        catch(tf2::TransformException &ex) 
        {
            ROS_WARN("Failure %s\n", ex.what()); //Print exception which was caught
            ROS_WARN("%s", error_msg.c_str()); //Print extended error message
            ros::WallDuration(1.0).sleep();
            transform_exception = true;
        }
        
    } while (transform_exception);

    return time_span;
    
}

transform_lib::Transform::Transform()
{

}

transform_lib::Transform::Transform(std::string &target_frame, std::string &source_frame, tf2_ros::Buffer* tfBuffer, double time_offset, ros::Duration time_out, std::string error_msg) : target_frame_(target_frame), source_frame_(source_frame), tfBuffer_(tfBuffer), time_offset_(time_offset), time_out_(time_out), error_msg_(error_msg)
{
    performInitialChecks();
}


transform_lib::Transform::Transform(std::string &target_frame, std::string &source_frame, tf2_ros::Buffer* tfBuffer, double time_offset, std::string error_msg) : target_frame_(target_frame), source_frame_(source_frame), tfBuffer_(tfBuffer), time_offset_(time_offset), error_msg_(error_msg)
{
    time_out_ = ros::Duration(0.0);
    performInitialChecks();
}

transform_lib::Transform::Transform(std::string &target_frame, std::string &source_frame, tf2_ros::Buffer* tfBuffer, std::string error_msg) : target_frame_(target_frame), source_frame_(source_frame), tfBuffer_(tfBuffer), error_msg_(error_msg)
{
    time_out_ = ros::Duration(0.0);
    time_offset_ = 0;
    performInitialChecks();
}

transform_lib::Transform::Transform(std::string &target_frame, std::string &source_frame, tf2_ros::Buffer* tfBuffer)
{
    time_out_ = ros::Duration(0.0);
    time_offset_ = 0;
    error_msg_ = "Cannot find transform!";
    tfBuffer_->_frameExists(target_frame_);
    performInitialChecks();
}

void transform_lib::Transform::initialize(std::string &target_frame, std::string &source_frame, tf2_ros::Buffer* tfBuffer, double time_offset, ros::Duration time_out, std::string error_msg) 
{
    target_frame_ = target_frame;
    source_frame_ = source_frame;
    tfBuffer_ = tfBuffer;
    time_offset_ = 0;
    time_out_ = time_out;
    error_msg_ = error_msg;
    performInitialChecks();
}

void transform_lib::Transform::initialize(std::string &target_frame, std::string &source_frame, tf2_ros::Buffer* tfBuffer, double time_offset, std::string error_msg) 
{
    target_frame_ = target_frame;
    source_frame_ = source_frame;
    tfBuffer_ = tfBuffer;
    time_offset_ = 0 ;
    error_msg_ = error_msg;
    time_out_ = ros::Duration(0.0);
    performInitialChecks();
}

void transform_lib::Transform::initialize(std::string &target_frame, std::string &source_frame, tf2_ros::Buffer* tfBuffer, std::string error_msg) 
{
    target_frame_ = target_frame;
    source_frame_ = source_frame;
    tfBuffer_ = tfBuffer;
    error_msg_ = error_msg;
    time_out_ = ros::Duration(0.0);
    time_offset_ = 0;
    performInitialChecks();
}

void transform_lib::Transform::initialize(std::string &target_frame, std::string &source_frame, tf2_ros::Buffer* tfBuffer)
{
    time_out_ = ros::Duration(0.0);
    time_offset_ = 0;
    error_msg_ = "Cannot find transform!";
    performInitialChecks();
}

void transform_lib::Transform::getTransform(geometry_msgs::TransformStamped &transform_result)
{
    bool transform_exception;
    do
    {
        transform_exception = false;
        try
        {
            transform_result = tfBuffer_ -> lookupTransform(target_frame_, source_frame_, ros::Time(time_offset_), time_out_);
        }
        catch(tf2::TransformException &ex) 
        {
            ROS_WARN("Failure %s\n", ex.what()); //Print exception which was caught
            ROS_WARN("%s", error_msg_.c_str()); //Print extended error message
            ros::WallDuration(1.0).sleep();
            transform_exception = true;
        }
        
    } while (transform_exception);
}

void transform_lib::Transform::getTransformTimed(geometry_msgs::TransformStamped &transform_result)
{
    int64_t time_span = -1000;
    bool transform_exception;
    do
    {
        transform_exception = false;
        try
        {
            std::chrono::high_resolution_clock::time_point t1 = std::chrono::high_resolution_clock::now();

            transform_result = tfBuffer_ -> lookupTransform(target_frame_, source_frame_, ros::Time(time_offset_), time_out_);

            std::chrono::high_resolution_clock::time_point t2 = std::chrono::high_resolution_clock::now();
            time_span = std::chrono::duration_cast<std::chrono::microseconds>(t2 - t1).count();
        }
        catch(tf2::TransformException &ex) 
        {
            ROS_WARN("Failure %s\n", ex.what()); //Print exception which was caught
            ROS_WARN("%s", error_msg_.c_str()); //Print extended error message
            ros::WallDuration(1.0).sleep();
            transform_exception = true;
        }
    } while (transform_exception);
    ROS_INFO("Time to find transform between %s and %s was: %i ms", target_frame_.c_str(), source_frame_.c_str(), (int)time_span);
}

void transform_lib::Transform::performInitialChecks()
{
    {
        if(target_frame_.empty() || source_frame_.empty())
        {
            ROS_ERROR("Tf frames cannot be an empty string!");
        }
        bool cannot_find_frames;
        do
        {
            ros::WallDuration(0.3).sleep();
            cannot_find_frames = false;
            if(!tfBuffer_-> _frameExists(target_frame_))
            {
                ROS_WARN("%s frame is required but does not yet exist", target_frame_.c_str());
                ros::WallDuration(1.0).sleep();
                cannot_find_frames = true;
            }
            if(!tfBuffer_-> _frameExists(source_frame_))
            {
                ROS_WARN("%s frame is required but does not yet exist", source_frame_.c_str());
                ros::WallDuration(1.0).sleep();
                cannot_find_frames = true;
            }
        } while (cannot_find_frames);
    }
    initialized_ = true;
}