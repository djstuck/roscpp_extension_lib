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
                ROS_WARN("%s", error_msg.c_str()); //Print extended error message
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
     * @param time 
     * @param duration 
     * @param error_msg 
     * @return time in milliseconds 
     */
    int getTransformTimed(std::string &target_frame, std::string &source_frame, tf2_ros::Buffer* tfBuffer, geometry_msgs::TransformStamped &transform_result, ros::Time &time, ros::Duration &duration, std::string &error_msg)
    {
        int64_t time_span = -1000;
        bool transform_exception;
        do
        {

            transform_exception = false;
            try
            {
                std::chrono::high_resolution_clock::time_point t1 = std::chrono::high_resolution_clock::now();

                transform_result = tfBuffer -> lookupTransform(target_frame, source_frame, time, duration);

                std::chrono::high_resolution_clock::time_point t2 = std::chrono::high_resolution_clock::now();
                time_span = std::chrono::duration_cast<std::chrono::microseconds>(t2 - t1).count();
            }
            catch(tf2::TransformException &ex) 
            {
                ROS_WARN("Failure %s\n", ex.what()); //Print exception which was caught
                ROS_WARN("%s", error_msg.c_str()); //Print extended error message
                ros::Duration(1.0).sleep();
                transform_exception = true;
            }
            
        } while (transform_exception);

        return time_span;
        
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
     * @return time in milliseconds
     */
    inline int getTransformTimed(std::string &target_frame, std::string &source_frame, tf2_ros::Buffer* tfBuffer, geometry_msgs::TransformStamped &transform_result, ros::Time &time, std::string &error_msg)
    {
        ros::Duration duration(0);
        return getTransformTimed(target_frame, source_frame, tfBuffer, transform_result, time, duration, error_msg);
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
        ros::Time time = ros::Time::now();
        ros::Duration duration(0);
        return getTransformTimed(target_frame, source_frame, tfBuffer, transform_result, time, duration, error_msg);
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
        ros::Time time = ros::Time::now();
        ros::Duration duration(0);
        std::string error_msg = "Failed to get transform";
        return getTransformTimed(target_frame, source_frame, tfBuffer, transform_result, time, duration, error_msg);
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

    inline void getTransformToEigen(std::string &target_frame, std::string &source_frame, tf2_ros::Buffer* tfBuffer, Eigen::Matrix4f& result, ros::Time &time, ros::Duration &duration, std::string &error_msg)
    {
        geometry_msgs::TransformStamped transform_result;
        getTransform(target_frame, source_frame, tfBuffer, transform_result, time, duration, error_msg);
        transformToEigen(transform_result, result);
    }

    inline void getTransformToEigen(std::string &target_frame, std::string &source_frame, tf2_ros::Buffer* tfBuffer, Eigen::Matrix4f& result, ros::Time &time, std::string &error_msg)
    {
        geometry_msgs::TransformStamped transform_result;
        getTransform(target_frame, source_frame, tfBuffer, transform_result, time, error_msg);
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

    inline int getTransformToEigenTimed(std::string &target_frame, std::string &source_frame, tf2_ros::Buffer* tfBuffer, Eigen::Matrix4f& result, ros::Time &time, ros::Duration &duration, std::string &error_msg)
    {
        geometry_msgs::TransformStamped transform_result;
        getTransformTimed(target_frame, source_frame, tfBuffer, transform_result, time, duration, error_msg);
        transformToEigen(transform_result, result);
    }

    inline int getTransformToEigenTimed(std::string &target_frame, std::string &source_frame, tf2_ros::Buffer* tfBuffer, Eigen::Matrix4f& result, ros::Time &time, std::string &error_msg)
    {
        geometry_msgs::TransformStamped transform_result;
        getTransformTimed(target_frame, source_frame, tfBuffer, transform_result, time, error_msg);
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
        ros::Time time_;
        ros::Duration time_out_;
        std::string error_msg_;

    public:
        Transform(std::string &target_frame, std::string &source_frame, tf2_ros::Buffer* tfBuffer, ros::Time time, ros::Duration time_out, std::string error_msg) : target_frame_(target_frame), source_frame_(source_frame), tfBuffer_(tfBuffer), time_(time), time_out_(time_out), error_msg_(error_msg)
        {
            performInitialChecks();
        }

        Transform(std::string &target_frame, std::string &source_frame, tf2_ros::Buffer* tfBuffer, ros::Time time, std::string error_msg) : target_frame_(target_frame), source_frame_(source_frame), tfBuffer_(tfBuffer), time_(time), error_msg_(error_msg)
        {
            time_out_ = ros::Duration(0.0);
            performInitialChecks();
        }

        Transform(std::string &target_frame, std::string &source_frame, tf2_ros::Buffer* tfBuffer, std::string error_msg) : target_frame_(target_frame), source_frame_(source_frame), tfBuffer_(tfBuffer), error_msg_(error_msg)
        {
            time_out_ = ros::Duration(0.0);
            time_ = ros::Time::now();
            performInitialChecks();
        }

        Transform(std::string &target_frame, std::string &source_frame, tf2_ros::Buffer* tfBuffer)
        {
            time_out_ = ros::Duration(0.0);
            time_ = ros::Time::now();
            error_msg_ = "Cannot find transform!";
            tfBuffer_->_frameExists(target_frame_);
            performInitialChecks();
        }

        void getTransform(geometry_msgs::TransformStamped &transform_result)
        {
            bool transform_exception;
            do
            {
                transform_exception = false;
                try
                {
                    transform_result = tfBuffer_ -> lookupTransform(target_frame_, source_frame_, time_, time_out_);
                }
                catch(tf2::TransformException &ex) 
                {
                    ROS_WARN("Failure %s\n", ex.what()); //Print exception which was caught
                    ROS_WARN("%s", error_msg.c_str()); //Print extended error message
                    ros::Duration(1.0).sleep();
                    transform_exception = true;
                }
                
            } while (transform_exception);
        }

        void getTransformTimed(geometry_msgs::TransformStamped &transform_result)
        {
            int64_t time_span = -1000;
            bool transform_exception;
            do
            {
                transform_exception = false;
                try
                {
                    std::chrono::high_resolution_clock::time_point t1 = std::chrono::high_resolution_clock::now();

                    transform_result = tfBuffer_ -> lookupTransform(target_frame_, source_frame_, time_, time_out_);

                    std::chrono::high_resolution_clock::time_point t2 = std::chrono::high_resolution_clock::now();
                    time_span = std::chrono::duration_cast<std::chrono::microseconds>(t2 - t1).count();
                }
                catch(tf2::TransformException &ex) 
                {
                    ROS_WARN("Failure %s\n", ex.what()); //Print exception which was caught
                    ROS_WARN("%s", error_msg.c_str()); //Print extended error message
                    ros::Duration(1.0).sleep();
                    transform_exception = true;
                }
                
            } while (transform_exception);
            ROS_INFO("Time to find transform between %s and %s was: %i ms", target_frame_, source_frame_, time_span);
        }

    private:
        void performInitialChecks()
        {
            if(!tfBuffer_ == nullptr)
            {
                bool cannot_find_frames = false;
                do
                {
                    if(!tfBuffer_-> _frameExists(target_frame_))
                    {
                        ROS_WARN("%s frame is required but does not exist", target_frame_.c_str());
                        ros::Duration(1.0).sleep();
                        cannot_find_frames = true;
                    }
                    if(!tfBuffer_-> _frameExists(source_frame_))
                    {
                        ROS_WARN("%s frame is required but does not exist", source_frame_.c_str());
                        ros::Duration(1.0).sleep();
                        cannot_find_frames = true;
                    }
                } while (cannot_find_frames);
            }
            else
            {
                ROS_ERROR("tf2_ros::Buffer was not initialized properly.");
            }
        }

    };

}  // namespace transform_lib
