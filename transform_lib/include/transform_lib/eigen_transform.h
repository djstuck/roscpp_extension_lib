#pragma once
#include <eigen3/Eigen/Dense>
#include "geometry_msgs/Transform.h"
#include "geometry_msgs/TransformStamped.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/PoseStamped.h"

namespace transform_lib
{

/**
 * @brief Converts a transform to a transformation matrix.
 * 
 * @param t Transform to be converted.
 * @return Eigen::Matrix4f 
 */
inline Eigen::Matrix4f transformToEigen(geometry_msgs::Transform &t)
{
    const geometry_msgs::Quaternion &q = t.rotation;
    const geometry_msgs::Vector3 &p = t.translation;
    Eigen::Matrix4f t_matrix;
    t_matrix <<     1 - 2*q.y*q.y - 2*q.z*q.z   , 2*q.x*q.y - 2*q.w*q.z       , 2*q.x*q.z + 2*q.w*q.y      , p.x, \
                    2*q.x*q.y + 2*q.w*q.z       , 1 - 2*q.x*q.x - 2*q.z*q.z   , 2*q.y*q.z - 2*q.w*q.x      , p.y, \
                    2*q.x*q.z - 2*q.w*q.y       , 2*q.y*q.z + 2*q.w*q.x       , 1 - 2*q.x*q.x - 2*q.y*q.y  , p.z, \
                    0                           , 0                           , 0                          , 1;
    return t_matrix;
}

/**
 * @brief Converts a transform stamped to a transformation matrix.
 * 
 * @param t transform stamped to be converted.
 * @return Eigen::Matrix4f 
 */
inline Eigen::Matrix4f transformToEigen(geometry_msgs::TransformStamped& t)
{
    return transformToEigen(t.transform);
}

/**
 * @brief Multiplies a transformational matrix by a zero vector to retrieve the translational information from the transformation matrix
 * 
 * @return Eigen::Vector4f with the following structure: [x, y, z, 1]
 */
inline Eigen::Vector4f multiplyByZeroVector(Eigen::Matrix4f& m)
{
    Eigen::Vector4f v;
    v << 0, 0, 0, 1;
    return m * v;
}

/**
 * @brief Create a translation vector from known x, y, z data. This vector can be used to multiply with the transformation matrix
 * 
 * @param x 
 * @param y 
 * @param z 
 * @return Eigen::Vector4f 
 */
inline Eigen::Vector4f createTranslationVector(float& x, float& y, float& z)
{
    Eigen::Vector4f v;
    v << x, y, z, 1;
    return v;
}

/**
 * @brief Convert a pose to a transformation matrix. The orientation included in the transformation matrix is from the orientation component of the pose.
 * 
 * @param pose 
 * @return Eigen::Matrix4f 
 */
inline Eigen::Matrix4f poseToEigen(geometry_msgs::Pose& pose)
{
    const geometry_msgs::Point &p = pose.position;
    const geometry_msgs::Quaternion &q = pose.orientation;
    Eigen::Matrix4f t_matrix;
    t_matrix <<     1 - 2*q.y*q.y - 2*q.z*q.z   , 2*q.x*q.y - 2*q.w*q.z       , 2*q.x*q.z + 2*q.w*q.y      , p.x, \
                    2*q.x*q.y + 2*q.w*q.z       , 1 - 2*q.x*q.x - 2*q.z*q.z   , 2*q.y*q.z - 2*q.w*q.x      , p.y, \
                    2*q.x*q.z - 2*q.w*q.y       , 2*q.y*q.z + 2*q.w*q.x       , 1 - 2*q.x*q.x - 2*q.y*q.y  , p.z, \
                    0                           , 0                           , 0                          , 1;
    return t_matrix;
}

/**
 * @brief Convert a PoseStamped to a transformation matrix. The orientation included in the transformation matrix is from the orientation component of the pose.
 * 
 * @param pose 
 * @return Eigen::Matrix4f 
 */
inline Eigen::Matrix4f poseToEigen(geometry_msgs::PoseStamped& pose)
{
    return poseToEigen(pose.pose);
}

inline geometry_msgs::Pose eigenToPoseMsg(Eigen::Matrix4f &m)
{
    geometry_msgs::Pose pose;
    pose.position.x = m.coeff(3);
    pose.position.y = m.coeff(7);
    pose.position.z = m.coeff(11);

    return pose;
}

} // namespace