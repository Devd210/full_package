#pragma once
#ifndef LICENSE_HPP
#define LICENSE_HPP

#include <dirent.h>
#include <unistd.h>
#include <pwd.h>

#include <sys/stat.h>
#include <sys/types.h>

#include <iostream>
#include <fstream>

#include <ros/ros.h>

#include <jwt-cpp/jwt.h>
#include <picojson/picojson.h>
#include <cpr/cpr.h>

class License {
 private:
  // public key to be used to decode the license
  const std::string pub_key_;
  // public key to be used to decode the license
  const std::string version_;
  // the url to get the license from
  const std::string url_;
  // initials of the file name in which JWT token is stored
  const std::string lic_file_name_;
  // initials of the file name in which credentials are stored
  const std::string cred_file_name_;
  // the default directory where files are stored
  const std::string dir_name_;
  // the full directory where file is stored or read from
  std::string full_dir_;

  std::string email_;
  std::string username_;
  std::string password_;
  std::string robot_name_;

  //brief: returns all the mac addresses stored in /sys/class/net/
  static std::vector<std::string> getMacAddresses();

  //brief: reads data from a file
  static bool readFromFile(std::string &, const std::string &);

  //brief: get license from server
  bool getLicense();

  //brief: checks if a file or directory exists
  static bool doesPathExist(const std::string &);

  //brief: checks JWT license string is valid license string
  bool verifyToken();

  //brief: takes in email and password from credentials_
  bool getCredentials();

  //brief: sends request to the server and receives the license string
  cpr::Response requestLicense(const std::string &);
 public:
  //brief: initialises default .mowito directory
  License();

  //brief: gets and verifies license as required
  bool verifyLicense();
};

#endif
