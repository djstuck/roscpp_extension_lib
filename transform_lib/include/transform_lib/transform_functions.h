#pragma once
#include "geometry_msgs/TransformStamped.h"
#include "tf2_ros/buffer.h"

namespace transform_lib
{
    /********************************************************************************************************************************
     * 
     * GetTransform functions
     * These functions extend the existing functionality of tf2_ros::Buffer::getTransform by implementing in a while loop with
     * transform exception handling and added error messages.
     * 
     ********************************************************************************************************************************/

    /**
     * @brief Get a transform from two frame ids
     * 
     * @param target_frame 
     * @param source_frame 
     * @param tfBuffer 
     * @param transform_result 
     * @param time 
     * @param duration 
     * @param error_msg 
     */
    void getTransform(std::string &target_frame, std::string &source_frame, tf2_ros::Buffer* tfBuffer, geometry_msgs::TransformStamped &transform_result, ros::Time &time, ros::Duration &duration, std::string &error_msg)
    {
        bool transform_exception;
        do
        {
            transform_exception = false;
            try
            {
                transform_result = tfBuffer -> lookupTransform(target_frame, source_frame, time, duration);
            }
            catch(tf2::TransformException &ex) 
            {
                ROS_WARN("Failure %s\n", ex.what()); //Print exception which was caught
                ROS_WARN("%s", error_msg); //Print extended error message
                ros::Duration(1.0).sleep();
                transform_exception = true;
            }
            
        } while (transform_exception);
        
    }

    /**
     * @brief Get a transform from two frame ids.
     * 
     * @param target_frame 
     * @param source_frame 
     * @param tfBuffer 
     * @param transform_result 
     * @param time 
     * @param error_msg 
     */
    inline void getTransform(std::string &target_frame, std::string &source_frame, tf2_ros::Buffer* tfBuffer, geometry_msgs::TransformStamped &transform_result, ros::Time &time, std::string &error_msg)
    {
        ros::Duration duration(0);
        getTransform(target_frame, source_frame, tfBuffer, transform_result, time, duration, error_msg);
    }

    /**
     * @brief Get a transform from two frame ids.
     * 
     * @param target_frame 
     * @param source_frame 
     * @param tfBuffer 
     * @param transform_result 
     * @param error_msg 
     */
    inline void getTransform(std::string &target_frame, std::string &source_frame, tf2_ros::Buffer* tfBuffer, geometry_msgs::TransformStamped &transform_result,  std::string &error_msg)
    {
        ros::Time time = ros::Time::now();
        ros::Duration duration(0);
        getTransform(target_frame, source_frame, tfBuffer, transform_result, time, duration, error_msg);
    }

    /**
     * @brief Get a transform from two frame ids.
     * 
     * @param target_frame 
     * @param source_frame 
     * @param tfBuffer 
     * @param transform_result 
     */
    inline void getTransform(std::string &target_frame, std::string &source_frame, tf2_ros::Buffer* tfBuffer, geometry_msgs::TransformStamped &transform_result)
    {
        ros::Time time = ros::Time::now();
        ros::Duration duration(0);
        std::string error_msg = "Failed to get transform";
        getTransform(target_frame, source_frame, tfBuffer, transform_result, time, duration, error_msg);
    }

    /**
     * @brief Get a transform from two frame ids.
     * 
     * @param target_frame 
     * @param source_frame 
     * @param tfBuffer 
     * @param time 
     * @param duration 
     * @param error_msg 
     * @return geometry_msgs::TransformStamped 
     */
    inline geometry_msgs::TransformStamped getTransform(std::string &target_frame, std::string &source_frame, tf2_ros::Buffer* tfBuffer, ros::Time &time, ros::Duration &duration, std::string &error_msg)
    {
        geometry_msgs::TransformStamped transform_result;
        getTransform(target_frame, source_frame, tfBuffer, transform_result, time, duration, error_msg);
        return transform_result;
    }

    /**
     * @brief Get a transform from two frame ids.
     * 
     * @param target_frame 
     * @param source_frame 
     * @param tfBuffer 
     * @param time 
     * @param error_msg 
     * @return geometry_msgs::TransformStamped 
     */
    inline geometry_msgs::TransformStamped getTransform(std::string &target_frame, std::string &source_frame, tf2_ros::Buffer* tfBuffer, ros::Time &time, std::string &error_msg)
    {
        geometry_msgs::TransformStamped transform_result;
        getTransform(target_frame, source_frame, tfBuffer, transform_result, time, error_msg);
        return transform_result;
    }

    /**
     * @brief Get a transform from two frame ids.
     * 
     * @param target_frame 
     * @param source_frame 
     * @param tfBuffer 
     * @param error_msg 
     * @return geometry_msgs::TransformStamped 
     */
    inline geometry_msgs::TransformStamped getTransform(std::string &target_frame, std::string &source_frame, tf2_ros::Buffer* tfBuffer, std::string &error_msg)
    {
        geometry_msgs::TransformStamped transform_result;
        getTransform(target_frame, source_frame, tfBuffer, transform_result, error_msg);
        return transform_result;
    }

    /**
     * @brief Get a transform from two frame ids.
     * 
     * @param target_frame 
     * @param source_frame 
     * @param tfBuffer 
     * @return geometry_msgs::TransformStamped 
     */
    inline geometry_msgs::TransformStamped getTransform(std::string &target_frame, std::string &source_frame, tf2_ros::Buffer* tfBuffer)
    {
        geometry_msgs::TransformStamped transform_result;
        getTransform(target_frame, source_frame, tfBuffer, transform_result);
        return transform_result;
    }
}