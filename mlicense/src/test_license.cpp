#include <mlicense/license.hpp>

int main() {
  License lic;

  ROS_INFO("Checking License...");
  if (lic.verifyLicense()) {
    ROS_INFO("Success!!!");
  } else {
    ROS_ERROR("License could not be verified.");
  }

}