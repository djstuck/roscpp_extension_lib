#pragma once
#include <eigen3/Eigen/Dense>
#include "geometry_msgs/Transform.h"
#include "geometry_msgs/TransformStamped.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/PoseStamped.h"

namespace transform_lib
{
    /********************************************************************************************************************************
     * 
     * Common Defines that will be used throughout this file
     * 
     ********************************************************************************************************************************/

    /**
     * @brief This define the code needed to create a 4x4 transformation matrix from a quaternion called q and a vector3/point called p;
     * @details I created this define as to not create another level of functions which could create a performance difference.
     * 
     */
    #define quat_and_vector3_to_matrix  1 - 2*q.y*q.y - 2*q.z*q.z   , 2*q.x*q.y - 2*q.w*q.z       , 2*q.x*q.z + 2*q.w*q.y      , p.x, \
                                        2*q.x*q.y + 2*q.w*q.z       , 1 - 2*q.x*q.x - 2*q.z*q.z   , 2*q.y*q.z - 2*q.w*q.x      , p.y, \
                                        2*q.x*q.z - 2*q.w*q.y       , 2*q.y*q.z + 2*q.w*q.x       , 1 - 2*q.x*q.x - 2*q.y*q.y  , p.z, \
                                        0                           , 0                           , 0                          , 1

    /**
     * @brief Define the coefficients of x, y, and z from a 4x4 transformation matrix. This is commonly used to retrieve the x , y, and z values from the transformation matrix without any extra processes like multiplyByZeroVector.
     * 
     */
    #define T_MATRIX_X coeff(12)
    #define T_MATRIX_Y coeff(13)
    #define T_MATRIX_Z coeff(14)

    /********************************************************************************************************************************
     * 
     * Conversions from a geometry_msgs::Transform/TransformStamped to a Eigen::Matrix4f
     * 
     ********************************************************************************************************************************/

    /**
     * @brief Converts a transform to a transformation matrix.
     * 
     * @param t Transform to be converted.
     * @return Eigen::Matrix4f containing the transformation matrix
     */
    inline Eigen::Matrix4f transformToEigen(geometry_msgs::Transform &t)
    {
        const geometry_msgs::Quaternion &q = t.rotation;
        const geometry_msgs::Vector3 &p = t.translation;
        Eigen::Matrix4f t_matrix;
        t_matrix <<  quat_and_vector3_to_matrix;
        return t_matrix;
    }

    /**
     * @brief Converts a transform stamped to a transformation matrix.
     * 
     * @param t transform stamped to be converted.
     * @return Eigen::Matrix4f containing the transformation matrix
     */
    inline Eigen::Matrix4f transformToEigen(geometry_msgs::TransformStamped& t)
    {
        return transformToEigen(t.transform);
    }

    /**
     * @brief Converts a transform to a transformation matrix. This implementation avoids copying the transformation matrix during the return.
     * 
     * @param t transform to be converted.
     * @param result Eigen::Matrix4f containing the transformation matrix
     */
    inline void transformToEigen(geometry_msgs::Transform &t, Eigen::Matrix4f &result)
    {
        const geometry_msgs::Quaternion &q = t.rotation;
        const geometry_msgs::Vector3 &p = t.translation;
        result <<  quat_and_vector3_to_matrix;
    }

    /**
     * @brief Converts a transform stamped to a transformation matrix. This implementation avoids copying the transformation matrix during the return.
     * 
     * @param t transform stamped to be converted.
     * @param result Eigen::Matrix4f containing the transformation matrix
     */
    inline void transformToEigen(geometry_msgs::TransformStamped& t, Eigen::Matrix4f& result)
    {
        transformToEigen(t.transform, result);
    }

    /********************************************************************************************************************************
     * 
     * Multiplication of a transformation matrix by a zero vector to produce a vector containing x, y, z data
     * 
     ********************************************************************************************************************************/

    /**
     * @brief Multiplies a transformational matrix by a zero vector to retrieve the translational information from the transformation matrix
     * 
     * @param m Transformation matrix to be multiplied by the zero vector
     * @return Eigen::Vector4f with the following structure: [x, y, z, 1]
     */
    inline Eigen::Vector4f multiplyByZeroVector(Eigen::Matrix4f& m)
    {
        Eigen::Vector4f v;
        v << 0, 0, 0, 1;
        return m * v;
    }

    /**
     * @brief Multiplies a transformational matrix by a zero vector to retrieve the translational information from the transformation matrix. This implementation avoids copying the vector during the return.
     * 
     * @param m Transformation matrix to be multiplied by the zero vector;
     * @param result Eigen::Vector4f containing the translational information with the following structure: [x, y, z, 1]
     */
    inline void multiplyByZeroVector(Eigen::Matrix4f& m, Eigen::Vector4f& result)
    {
        Eigen::Vector4f v;
        v << 0, 0, 0, 1;
        result = m * v;
    }

    /********************************************************************************************************************************
     * 
     * Creating a translation vector from x, y, z, data
     * 
     ********************************************************************************************************************************/

    /**
     * @brief Create a translation vector from known x, y, z data. This vector can be used to multiply with the transformation matrix
     * 
     * @param x 
     * @param y 
     * @param z 
     * @return Eigen::Vector4f with the following structure: [x, y, z, 1]
     */
    inline Eigen::Vector4f createTranslationVector(float& x, float& y, float& z)
    {
        Eigen::Vector4f v;
        v << x, y, z, 1;
        return v;
    }

    /**
     * @brief Create a translation vector from known x, y, z data. This vector can be used to multiply with the transformation matrix. This implementation avoids copying the vector during the return
     * 
     * @param x 
     * @param y 
     * @param z 
     * @param result Eigen::Vector4f with the following structure: [x, y, z, 1]
     */
    inline void createTranslationVector(float& x, float& y, float& z, Eigen::Vector4f& result)
    {
        result << x, y, z, 1;
    }

    /********************************************************************************************************************************
     * 
     * Conversions from a geometry_msgs::Pose/PoseStamped to Eigen::Matrix4f transformation matrix.
     * This is commonly used to transform the position and orientation of a pose into a global frame by using matrix multiplication.
     * 
     ********************************************************************************************************************************/

    /**
     * @brief Convert a pose to a transformation matrix. The orientation included in the transformation matrix is from the orientation component of the pose.
     * 
     * @param pose 
     * @return Eigen::Matrix4f containing the transformation matrix
     */
    inline Eigen::Matrix4f poseToEigen(geometry_msgs::Pose& pose)
    {
        const geometry_msgs::Point &p = pose.position;
        const geometry_msgs::Quaternion &q = pose.orientation;
        Eigen::Matrix4f t_matrix;
        t_matrix <<  quat_and_vector3_to_matrix;
        return t_matrix;
    }

    /**
     * @brief Convert a PoseStamped to a transformation matrix. The rotation component included in the transformation matrix is from the orientation component of the pose.
     * 
     * @param pose 
     * @return Eigen::Matrix4f containing the transformation matrix
     */
    inline Eigen::Matrix4f poseToEigen(geometry_msgs::PoseStamped& pose)
    {
        return poseToEigen(pose.pose);
    }

    /**
     * @brief Convert a pose to a transformation matrix. The rotation component included in the transformation matrix is from the orientation component of the pose. This implementation avoids copying the matrix during the return.
     * 
     * @param pose 
     * @param result Eigen::Matrix4f containing the transformation matrix
     */
    inline void poseToEigen(geometry_msgs::Pose& pose, Eigen::Matrix4f& result)
    {
        const geometry_msgs::Point &p = pose.position;
        const geometry_msgs::Quaternion &q = pose.orientation;
        result <<  quat_and_vector3_to_matrix;
    }

    /**
     * @brief Convert a PoseStamped to a transformation matrix. The rotation component included in the transformation matrix is from the orientation component of the pose. This implementation avoids copying the matrix during the return.
     * 
     * @param pose 
     * @param Eigen::Matrix4f containing the transformation matrix
     */
    inline void poseToEigen(geometry_msgs::PoseStamped& pose, Eigen::Matrix4f& result)
    {
        poseToEigen(pose.pose, result);
    }

    /********************************************************************************************************************************
     * 
     * Conversions from a Eigen::Matrix4f transformation matrix to a geometry_msgs::Pose/PoseStamped
     * This is commonly used to transform the position and orientation of a pose into a global frame by using matrix multiplication.
     * 
     ********************************************************************************************************************************/

    inline void eigenToPoint(Eigen::Matrix4f &m, geometry_msgs::Point &pt)
    {
        pt.x = m.T_MATRIX_X;
        pt.y = m.T_MATRIX_Y;
        pt.z = m.T_MATRIX_Z;
    }

    inline void eigenToQuaternion(Eigen::Matrix4f &m, geometry_msgs::Quaternion &q)
    {
        float *a = m.data(); // convert eigen matrix to float array
        float trace = a[0] + a[5] + a[10];
        if( trace > 0 ) {
            float s = 0.5f / sqrtf(trace+ 1.0f);
            q.w = 0.25f / s;
            q.x = ( a[9] - a[6] ) * s;
            q.y = ( a[2] - a[8] ) * s;
            q.z = ( a[4] - a[1] ) * s;
        } else {
            if ( a[0] > a[5] && a[0] > a[10] ) {
            float s = 2.0f * sqrtf( 1.0f + a[0] - a[5] - a[10]);
            q.w = (a[9] - a[6] ) / s;
            q.x = 0.25f * s;
            q.y = (a[1] + a[4] ) / s;
            q.z = (a[2] + a[8] ) / s;
            } else if (a[5] > a[10]) {
            float s = 2.0f * sqrtf( 1.0f + a[5] - a[0] - a[10]);
            q.w = (a[2] - a[8] ) / s;
            q.x = (a[1] + a[4] ) / s;
            q.y = 0.25f * s;
            q.z = (a[6] + a[9] ) / s;
            } else {
            float s = 2.0f * sqrtf( 1.0f + a[10] - a[0] - a[5] );
            q.w = (a[4] - a[1] ) / s;
            q.x = (a[2] + a[8] ) / s;
            q.y = (a[6] + a[9] ) / s;
            q.z = 0.25f * s;
            }
        }
    }
    
    /**
     * @brief Convert a pose to a transformation matrix. The rotation component included
     * 
     * @param m 
     * @return geometry_msgs::Pose 
     */
    inline geometry_msgs::Pose eigenToPoseMsg(Eigen::Matrix4f &m)
    {
        geometry_msgs::Pose pose;
        eigenToPoint(m, pose.position);
        eigenToQuaternion(m, pose.orientation);

        return pose;
    }



} // namespace