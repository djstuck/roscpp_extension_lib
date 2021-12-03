#pragma once
#include <ros/ros.h>
#include <string>

/**
 * @brief This class adds extra functionality to the ros parameter server.
 * Extra functionality includes the following;
 *  - provide extra information to the user regarding the parameters loaded
 *  - set default parameters if parameters are not used
 *  - set predefined namespace to parameters
 * 
 * Motivation for this is to clean up code containing large amounts of parameters.
 * 
 */
class NodeParamLoader
{
public:

    /**
     * @brief Default constructor
     * @param nh nodehandle of the parameter server
     * @param namespace namespace of the parameters
     * 
     */
    NodeParamLoader(ros::NodeHandle* nh, std::string& param_namespace);

    /**
     * @brief load a float parameter into the node
     * @param name name of the parameter,  std::string
     * @param storage storage of the parameter, float
     * @param default_value default value to apply if parameter is missing, float
     * 
     */
    void loadParam(std::string name, float& storage, float default_value);

    /**
     * @brief load a double parameter into the node
     * @param name name of the parameter, std::string
     * @param storage storage of the parameter, double
     * @param default_value default value to apply if parameter is missing double
     * 
     */
    void loadParam(std::string name, double& storage, double default_value);

    /**
     * @brief load a string parameter into the node
     * @param name name of the parameter, std::string
     * @param storage storage of the parameter, std::string
     * @param default_value default value to apply if parameter is missing, std::string
     * 
     */
    void loadParam(std::string name, std::string& storage, std::string default_value);

protected:
        ros::NodeHandle* nh_;
        std::string namespace_;
};
 