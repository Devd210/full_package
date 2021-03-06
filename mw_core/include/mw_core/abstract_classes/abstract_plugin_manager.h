#pragma once

#include <map>
#include <string>
#include <vector>
#include <XmlRpcException.h>

#include <boost/function.hpp>

namespace mw_core {

template<typename PluginType>
class AbstractPluginManager {

 public:

  typedef boost::function<typename PluginType::Ptr(const std::string &plugin)> loadPluginFunction;

  typedef boost::function<bool(const std::string &name, const typename PluginType::Ptr &plugin_ptr)> initPluginFunction;

  AbstractPluginManager(const std::string param_name,
                        const loadPluginFunction &loadPlugin,
                        const initPluginFunction &initPlugin)
      : param_name_(param_name), loadPlugin_(loadPlugin), initPlugin_(initPlugin) {}

  bool loadPlugins() {
    ros::NodeHandle private_nh("~");

    XmlRpc::XmlRpcValue plugin_param_list;
    if (!private_nh.getParam(param_name_, plugin_param_list)) {
      ROS_WARN_STREAM("No " << param_name_ << " plugins configured! - Use the param \"" << param_name_ << "\", "
                                                                                                          "which must be a list of tuples with a name and a type.");
      return false;
    }

    try {
      for (int i = 0; i < plugin_param_list.size(); i++) {
        XmlRpc::XmlRpcValue elem = plugin_param_list[i];

        std::string name = elem["name"];
        std::string type = elem["type"];

        if (plugins_.find(name) != plugins_.end()) {
          ROS_ERROR_STREAM("The plugin \"" << name << "\" has already been loaded! Names must be unique!");
          return false;
        }
        typename PluginType::Ptr plugin_ptr = loadPlugin_(type);
        if (plugin_ptr && initPlugin_(name, plugin_ptr)) {

          plugins_.insert(
              std::pair<std::string, typename PluginType::Ptr>(name, plugin_ptr));

          plugins_type_.insert(std::pair<std::string, std::string>(name, type)); // save name to type mapping
          names_.push_back(name);

          ROS_INFO_STREAM("The plugin with the type \"" << type << "\" has been loaded successfully under the name \""
                                                        << name << "\".");
        } else {
          ROS_ERROR_STREAM("Could not load the plugin with the name \""
                               << name << "\" and the type \"" << type << "\"!");
        }
      }
    }
    catch (XmlRpc::XmlRpcException &e) {
      ROS_ERROR_STREAM("Invalid parameter structure. The \"" << param_name_ << "\" parameter has to be a list of structs "
                                                             << "with fields \"name\" and \"type\" of !");
      ROS_ERROR_STREAM(e.getMessage());
      return false;
    }
    // is there any plugin in the map?
    return !plugins_.empty();
  }

  bool hasPlugin(const std::string &name) {
    return static_cast<bool>(plugins_.count(name)); // returns 1 or 0;
  }

  std::string getType(const std::string &name) {
    auto iter = plugins_type_.find(name);
    return iter->second;
  }

  const std::vector<std::string> &getLoadedNames() {
    return names_;
  }

  typename PluginType::Ptr getPlugin(const std::string &name) {
    typename std::map<std::string, typename PluginType::Ptr>::iterator new_plugin = plugins_.find(name);
    if (new_plugin != plugins_.end()) {
      ROS_DEBUG_STREAM("Found plugin with the name \"" << name << "\".");
      return new_plugin->second;
    } else {
      ROS_WARN_STREAM("The plugin with the name \"" << name << "\" has not yet been loaded!");
      return typename PluginType::Ptr(); // return null ptr
    }
  }

 protected:
  std::map<std::string, typename PluginType::Ptr> plugins_;
  std::map<std::string, std::string> plugins_type_;
  std::vector<std::string> names_;
  const std::string param_name_;
  const loadPluginFunction loadPlugin_;
  const initPluginFunction initPlugin_;
};

}
