#include <node_param_loader/node_param_loader.h>

NodeParamLoader::NodeParamLoader(ros::NodeHandle* nh, std::string& param_namespace) : nh_(nh), namespace_(param_namespace) 
{

}

void NodeParamLoader::loadParam(std::string& name, float& storage, float& default_value)
{
    if(!nh_->getParam(namespace_ + name, storage))
    {
        char* complete_name;
        *complete_name = *(namespace_ + name).c_str();
        ROS_ERROR("Failed to load param '%s'. Default: %f", complete_name, default_value);
        storage = default_value;
    }
}

void NodeParamLoader::loadParam(std::string& name, double& storage, double& default_value)
{
    if(!nh_->getParam(namespace_ + name, storage))
    {
        char* complete_name;
        *complete_name = *(namespace_ + name).c_str();
        ROS_ERROR("Failed to load param '%s'. Default: %f", complete_name, default_value);
        storage = default_value;
    }
}

void NodeParamLoader::loadParam(std::string& name, std::string& storage, std::string& default_value)
{
    if(!nh_->getParam(namespace_ + name, storage))
    {
        char *complete_name, *default_value_char;
        *complete_name = *(namespace_ + name).c_str();
        *default_value_char = *default_value.c_str();
        ROS_ERROR("Failed to load param '%s'. Default: %s", complete_name, default_value_char);
        storage = default_value;
    }
}
