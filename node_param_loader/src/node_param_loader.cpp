#include <node_param_loader/node_param_loader.h>

NodeParamLoader::NodeParamLoader(ros::NodeHandle* nh, std::string param_namespace) : nh_(nh), namespace_(param_namespace) 
{

}

void NodeParamLoader::loadParam(std::string name, float& storage, float default_value)
{
    std::string complete_name = namespace_ + name;
    if(!nh_->getParam(complete_name, storage))
    {
        ROS_ERROR("Failed to load param '%s'. Default: %f", complete_name.c_str(), default_value);
        storage = default_value;
    }
}

void NodeParamLoader::loadParam(std::string name, double& storage, double default_value)
{
    std::string complete_name = namespace_ + name;
    if(!nh_->getParam(complete_name, storage))
    {
        ROS_ERROR("Failed to load param '%s'. Default: %f", complete_name.c_str(), default_value);
        storage = default_value;
    }
}

void NodeParamLoader::loadParam(std::string name, std::string& storage, std::string default_value)
{
    std::string complete_name = namespace_ + name;
    if(!nh_->getParam(complete_name, storage))
    {
        ROS_ERROR("Failed to load param '%s'. Default: %s", complete_name.c_str(), default_value.c_str
        ());
        storage = default_value;
    }
}

void NodeParamLoader::loadParam(std::string name, int& storage, int default_value)
{
    std::string complete_name = namespace_ + name;
    if(!nh_->getParam(complete_name, storage))
    {
        ROS_ERROR("Failed to load param '%s'. Default: %i", complete_name.c_str(), default_value);
        storage = default_value;
    }
}
