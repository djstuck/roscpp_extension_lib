#pragma once
#include "transform_lib/eigen_transform_conversions.h"
#include "geometry_msgs/TransformStamped.h"
#include "tf2_ros/buffer.h"
#include <chrono>

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
     * @param time_offset 
     * @param duration 
     * @param error_msg 
     */
    void getTransform(std::string &target_frame, std::string &source_frame, tf2_ros::Buffer* tfBuffer, geometry_msgs::TransformStamped &transform_result, double& time_offset, ros::Duration &duration, std::string &error_msg);

    /**
     * @brief Get a transform from two frame ids.
     * 
     * @param target_frame 
     * @param source_frame 
     * @param tfBuffer 
     * @param transform_result 
     * @param time_offset 
     * @param error_msg 
     */
    inline void getTransform(std::string &target_frame, std::string &source_frame, tf2_ros::Buffer* tfBuffer, geometry_msgs::TransformStamped &transform_result, double&time_offset, std::string &error_msg)
    {
        ros::Duration duration(0);
        getTransform(target_frame, source_frame, tfBuffer, transform_result, time_offset, duration, error_msg);
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
        double time_offset = 0;
        ros::Duration duration(0);
        getTransform(target_frame, source_frame, tfBuffer, transform_result, time_offset, duration, error_msg);
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
        double time_offset = 0;
        ros::Duration duration(0);
        std::string error_msg = "Failed to get transform";
        getTransform(target_frame, source_frame, tfBuffer, transform_result, time_offset, duration, error_msg);
    }

    /**
     * @brief Get a transform from two frame ids.
     * 
     * @param target_frame 
     * @param source_frame 
     * @param tfBuffer 
     * @param time_offset 
     * @param duration 
     * @param error_msg 
     * @return geometry_msgs::TransformStamped 
     */
    inline geometry_msgs::TransformStamped getTransform(std::string &target_frame, std::string &source_frame, tf2_ros::Buffer* tfBuffer, double& time_offset, ros::Duration &duration, std::string &error_msg)
    {
        geometry_msgs::TransformStamped transform_result;
        getTransform(target_frame, source_frame, tfBuffer, transform_result, time_offset, duration, error_msg);
        return transform_result;
    }

    /**
     * @brief Get a transform from two frame ids.
     * 
     * @param target_frame 
     * @param source_frame 
     * @param tfBuffer 
     * @param time_offset 
     * @param error_msg 
     * @return geometry_msgs::TransformStamped 
     */
    inline geometry_msgs::TransformStamped getTransform(std::string &target_frame, std::string &source_frame, tf2_ros::Buffer* tfBuffer, double& time_offset, std::string &error_msg)
    {
        geometry_msgs::TransformStamped transform_result;
        getTransform(target_frame, source_frame, tfBuffer, transform_result, time_offset, error_msg);
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

    /********************************************************************************************************************************
     * 
     * GetTransform functions with built-in time recording for debug purposes
     * These functions extend the existing functionality of tf2_ros::Buffer::getTransform by implementing in a while loop with
     * transform exception handling, added error messages and time recording.
     * 
     ********************************************************************************************************************************/

    /**
     * @brief Get the Transform between two frames and record the time it takes to retrieve the transform
     * 
     * @param target_frame 
     * @param source_frame 
     * @param tfBuffer 
     * @param transform_result 
     * @param time_offset 
     * @param duration 
     * @param error_msg 
     * @return time in milliseconds 
     */
    int getTransformTimed(std::string &target_frame, std::string &source_frame, tf2_ros::Buffer* tfBuffer, geometry_msgs::TransformStamped &transform_result, double& time_offset, ros::Duration &duration, std::string &error_msg);
    
    /**
     * @brief Get a transform from two frame ids.
     * 
     * @param target_frame 
     * @param source_frame 
     * @param tfBuffer 
     * @param transform_result 
     * @param time_offset 
     * @param error_msg 
     * @return time in milliseconds
     */
    inline int getTransformTimed(std::string &target_frame, std::string &source_frame, tf2_ros::Buffer* tfBuffer, geometry_msgs::TransformStamped &transform_result, double& time_offset, std::string &error_msg)
    {
        ros::Duration duration(0);
        return getTransformTimed(target_frame, source_frame, tfBuffer, transform_result, time_offset, duration, error_msg);
    }

    /**
     * @brief Get a transform from two frame ids.
     * 
     * @param target_frame 
     * @param source_frame 
     * @param tfBuffer 
     * @param transform_result 
     * @param error_msg 
     * @return time in milliseconds
     */
    inline int getTransformTimed(std::string &target_frame, std::string &source_frame, tf2_ros::Buffer* tfBuffer, geometry_msgs::TransformStamped &transform_result,  std::string &error_msg)
    {
        double time_offset = 0;
        ros::Duration duration(0);
        return getTransformTimed(target_frame, source_frame, tfBuffer, transform_result, time_offset, duration, error_msg);
    }

    /**
     * @brief Get a transform from two frame ids.
     * 
     * @param target_frame 
     * @param source_frame 
     * @param tfBuffer 
     * @param transform_result
     * @return time in milliseconds 
     */
    inline int getTransformTimed(std::string &target_frame, std::string &source_frame, tf2_ros::Buffer* tfBuffer, geometry_msgs::TransformStamped &transform_result)
    {
        double time_offset = 0;
        ros::Duration duration(0);
        std::string error_msg = "Failed to get transform";
        return getTransformTimed(target_frame, source_frame, tfBuffer, transform_result, time_offset, duration, error_msg);
    }

    /********************************************************************************************************************************
     * 
     * getTransformToEigen functions that can get the transform between two frames and parse into an Eigen matrix from one function.
     * These functions extend the existing functionality of tf2_ros::Buffer::getTransform by implementing in a while loop with
     * transform exception handling, added error messages and conversion to Eigen Matirx. The benefit of these functions is so there
     * is less confusing code in node files as well as the required variables are created on the stack and destroyed once the Eigen
     * Matrix has be calculated. 
     * 
     ********************************************************************************************************************************/

    inline void getTransformToEigen(std::string &target_frame, std::string &source_frame, tf2_ros::Buffer* tfBuffer, Eigen::Matrix4f& result, double& time_offset, ros::Duration &duration, std::string &error_msg)
    {
        geometry_msgs::TransformStamped transform_result;
        getTransform(target_frame, source_frame, tfBuffer, transform_result, time_offset, duration, error_msg);
        transformToEigen(transform_result, result);
    }

    inline void getTransformToEigen(std::string &target_frame, std::string &source_frame, tf2_ros::Buffer* tfBuffer, Eigen::Matrix4f& result, double& time_offset, std::string &error_msg)
    {
        geometry_msgs::TransformStamped transform_result;
        getTransform(target_frame, source_frame, tfBuffer, transform_result, time_offset, error_msg);
        transformToEigen(transform_result, result);
    }

    inline void getTransformToEigen(std::string &target_frame, std::string &source_frame, tf2_ros::Buffer* tfBuffer, Eigen::Matrix4f& result, std::string &error_msg)
    {
        geometry_msgs::TransformStamped transform_result;
        getTransform(target_frame, source_frame, tfBuffer, transform_result, error_msg);
        transformToEigen(transform_result, result);
    }

    inline void getTransformToEigen(std::string &target_frame, std::string &source_frame, tf2_ros::Buffer* tfBuffer, Eigen::Matrix4f& result)
    {
        geometry_msgs::TransformStamped transform_result;
        getTransform(target_frame, source_frame, tfBuffer, transform_result);
        transformToEigen(transform_result, result);
    }

    inline int getTransformToEigenTimed(std::string &target_frame, std::string &source_frame, tf2_ros::Buffer* tfBuffer, Eigen::Matrix4f& result, double& time_offset, ros::Duration &duration, std::string &error_msg)
    {
        geometry_msgs::TransformStamped transform_result;
        getTransformTimed(target_frame, source_frame, tfBuffer, transform_result, time_offset, duration, error_msg);
        transformToEigen(transform_result, result);
    }

    inline int getTransformToEigenTimed(std::string &target_frame, std::string &source_frame, tf2_ros::Buffer* tfBuffer, Eigen::Matrix4f& result, double& time_offset, std::string &error_msg)
    {
        geometry_msgs::TransformStamped transform_result;
        getTransformTimed(target_frame, source_frame, tfBuffer, transform_result, time_offset, error_msg);
        transformToEigen(transform_result, result);
    }

    inline int getTransformToEigenTimed(std::string &target_frame, std::string &source_frame, tf2_ros::Buffer* tfBuffer, Eigen::Matrix4f& result, std::string &error_msg)
    {
        geometry_msgs::TransformStamped transform_result;
        getTransformTimed(target_frame, source_frame, tfBuffer, transform_result, error_msg);
        transformToEigen(transform_result, result);
    }

    inline int getTransformToEigenTimed(std::string &target_frame, std::string &source_frame, tf2_ros::Buffer* tfBuffer, Eigen::Matrix4f& result)
    {
        geometry_msgs::TransformStamped transform_result;
        getTransformTimed(target_frame, source_frame, tfBuffer, transform_result);
        transformToEigen(transform_result, result);
    }

    class Transform
    {
    private:
        tf2_ros::Buffer* tfBuffer_;
        std::string target_frame_, source_frame_;
        double time_offset_;
        ros::Duration time_out_;
        std::string error_msg_;
        bool initialized_ = false;

    public:
        Transform();

        Transform(std::string &target_frame, std::string &source_frame, tf2_ros::Buffer* tfBuffer, double time_offset, ros::Duration time_out, std::string error_msg);

        Transform(std::string &target_frame, std::string &source_frame, tf2_ros::Buffer* tfBuffer, double time_offset, std::string error_msg);

        Transform(std::string &target_frame, std::string &source_frame, tf2_ros::Buffer* tfBuffer, std::string error_msg);

        Transform(std::string &target_frame, std::string &source_frame, tf2_ros::Buffer* tfBuffer);
        void initialize(std::string &target_frame, std::string &source_frame, tf2_ros::Buffer* tfBuffer, double time_offset, ros::Duration time_out, std::string error_msg);

        void initialize(std::string &target_frame, std::string &source_frame, tf2_ros::Buffer* tfBuffer, double time_offset, std::string error_msg);

        void initialize(std::string &target_frame, std::string &source_frame, tf2_ros::Buffer* tfBuffer, std::string error_msg);

        void initialize(std::string &target_frame, std::string &source_frame, tf2_ros::Buffer* tfBuffer);

        void getTransform(geometry_msgs::TransformStamped &transform_result);

        void getTransformTimed(geometry_msgs::TransformStamped &transform_result);

        inline void getTransformToEigen(Eigen::Matrix4f &result)
        {
            geometry_msgs::TransformStamped transform_result;
            getTransform(transform_result);
            transformToEigen(transform_result, result);
        }

        inline void getTransformToEigenTimed(Eigen::Matrix4f &result)
        {
            geometry_msgs::TransformStamped transform_result;
            getTransformTimed(transform_result);
            transformToEigen(transform_result, result);
        }

    private:
        void performInitialChecks();

    };

}  // namespace transform_lib
