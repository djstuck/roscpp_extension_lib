#pragma once
#include <eigen3/Eigen/Dense>
#include "geometry_msgs/Transform.h"
#include "geometry_msgs/TransformStamped.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/PoseStamped.h"
#include <chrono>

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

    inline void eigenToQuaternion(const Eigen::Matrix4f &m, geometry_msgs::Quaternion &q)
    {
        // std::chrono::high_resolution_clock::time_point  t1 = std::chrono::high_resolution_clock::now();
        /*float *a = m.data(); // convert eigen matrix to float array
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
        }*/
        // std::chrono::high_resolution_clock::time_point  t2 = std::chrono::high_resolution_clock::now();
        // auto time_span1 = std::chrono::duration_cast<std::chrono::microseconds>(t2 - t1);
        // std::cout << "Time taken1: " << time_span1.count() << " microseconds" << std::endl;
        Eigen::Matrix3f m3 = m.block<3,3>(0,0);
        Eigen::Quaternionf q2(m3);
        // std::chrono::high_resolution_clock::time_point  t3 = std::chrono::high_resolution_clock::now();
        q2.normalize();
        q.x = q2.x();
        q.y = q2.y();
        q.z = q2.z();
        q.w = q2.w();
        // std::chrono::high_resolution_clock::time_point  t4 = std::chrono::high_resolution_clock::now();
        // auto time_span2 = std::chrono::duration_cast<std::chrono::microseconds>(t4 - t3);
        // std::cout << "Time taken2: " << time_span2.count() << " microseconds" << std::endl;

    // This is trying to use eigen code to quickly calculate the quaternion. It is not working. need to do as above is too slow.
    /*const typename Eigen::internal::nested_eval<Eigen::Matrix4f,2>::type mat(m);
    EIGEN_USING_STD_MATH(sqrt)
    // This algorithm comes from  "Quaternion Calculus and Fast Animation",
    // Ken Shoemake, 1987 SIGGRAPH course notes
    Eigen::Scalar t = mat.trace();
    if (t > Scalar(0))
    {
      t = sqrt(t + Scalar(1.0));
      q.w() = Scalar(0.5)*t;
      t = Scalar(0.5)/t;
      q.x() = (mat.coeff(2,1) - mat.coeff(1,2)) * t;
      q.y() = (mat.coeff(0,2) - mat.coeff(2,0)) * t;
      q.z() = (mat.coeff(1,0) - mat.coeff(0,1)) * t;
    }
    else
    {
      Eigen::Index i = 0;
      if (mat.coeff(1,1) > mat.coeff(0,0))
        i = 1;
      if (mat.coeff(2,2) > mat.coeff(i,i))
        i = 2;
      Eigen::Index j = (i+1)%3;
      Eigen::Index k = (j+1)%3;

      t = sqrt(mat.coeff(i,i)-mat.coeff(j,j)-mat.coeff(k,k) + Scalar(1.0));
      q.coeffs().coeffRef(i) = Scalar(0.5) * t;
      t = Scalar(0.5)/t;
      q.w() = (mat.coeff(k,j)-mat.coeff(j,k))*t;
      q.coeffs().coeffRef(j) = (mat.coeff(j,i)+mat.coeff(i,j))*t;
      q.coeffs().coeffRef(k) = (mat.coeff(k,i)+mat.coeff(i,k))*t;
    }*/
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

    inline void eigenToPoseMsg(Eigen::Matrix4f &m, geometry_msgs::Pose &pose)
    {
        eigenToPoint(m, pose.position);
        eigenToQuaternion(m, pose.orientation);
    }

    inline void eigenToXYZRPY(Eigen::Matrix4f &m, float &x, float &y, float &z, float &roll, float &pitch, float &yaw)
    {
        x = m.coeff(0,3);
        y = m.coeff(1,3);
        z = m.coeff(2,3);
        float sy = sqrt(m.coeff(0,0));
        if (!sy < 0.000001)
        {
            roll = atan2(m.coeff(2,1), m.coeff(2,2));
	        pitch = atan2(-m.coeff(2,0), sy);
	        yaw = atan2(m.coeff(1,0), m.coeff(0,0));
        }
        else
        {
            roll = atan2(-m.coeff(1,2), m.coeff(1,1));
	        pitch = atan2(-m.coeff(2,0), sy);
	        yaw = 0;
        }
    }

    inline void printTransformMatrix(Eigen::Matrix4f &m)
    {
        float roll, pitch, yaw, x, y, z;
        eigenToXYZRPY(m, x, y, z, roll, pitch, yaw);

        std::cout << "x: " << x << " y: " << y << " z: " << z << std::endl;
        std::cout << "roll: " << roll*180/M_PI << " degrees" << " pitch: " << pitch*180/M_PI << " degrees" << " yaw:" << yaw*180/M_PI << " degrees" << std::endl;
    }


    inline Eigen::Matrix4f createTransformationMatrix(float x, float y, float z, float yaw) 
    {
    Eigen::Vector3f trans;
    trans << x,y,z; 
    Eigen::Transform<float, 3, Eigen::Affine> t;
    t = Eigen::Translation<float, 3>(trans);
    t.rotate(Eigen::AngleAxis<float>(yaw, Eigen::Vector3f::UnitZ()));
    return t.matrix();
    }

    struct LineEquation
    {
        float b, m;
    };

    /**
     * @brief This functions computes the transformation matrix in the form of a eigen matrix.
     * The transform represents the first frame in the lines frame. The lines frame is represented as (0, 0) of the line is at the
     * origin of the frame and the x component is facing along the line.
     * Please note: The transformation matrix returned is the inverse of the transformation matrix of the line in the first frame.
     * 
     * @param line_equation 
     * @return Eigen::Matrix4f 
     */
    inline Eigen::Matrix4f lineEquationToEigen(LineEquation &line_equation)
    {
       return createTransformationMatrix(0, line_equation.b, 0, atan(line_equation.m)).inverse();
    }

    /**
     * @brief This function computes the 2D distance calculated from the translational component of the transformation matrix
     * Note: It uses the x and y components of the transformation matrix
     * 
     * @param m 
     * @return float distance
     */
    inline float eigen2DDistance(Eigen::Matrix4f& m)
    {
        return sqrt((m.T_MATRIX_X * m.T_MATRIX_X) + (m.T_MATRIX_Y * m.T_MATRIX_Y));
    }

    /**
     * @brief This function computes the 3D distance calculated from the translational component of the transformation matrix
     * Note: It uses the x, y and z components of the transformation matrix
     * 
     * @param m 
     * @return float distance
     */
    inline float eigen3DDistance(Eigen::Matrix4f& m)
    {
        return sqrt((m.T_MATRIX_X * m.T_MATRIX_X) + (m.T_MATRIX_Y * m.T_MATRIX_Y) + (m.T_MATRIX_Z * m.T_MATRIX_Z));
    }


} // namespace