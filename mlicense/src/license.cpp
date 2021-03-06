#include <mlicense/license.hpp>

License::License()
    :
    pub_key_{
/* begin public_key */ R"(-----BEGIN PUBLIC KEY-----
MIICIjANBgkqhkiG9w0BAQEFAAOCAg8AMIICCgKCAgEAoRplv49xbwge6s1xc2jy
NZDO6oWK7ak6g5OJ9T8pFH2VKfl/PitTXK49WPqWXXn9XirY+RpR6xQiSFv8a1S4
506UrboQGFo7S+Glsy6QxsGTZac9isEi+0AHl4a39faekVBiHDny9VG75VsxVNIK
g6224wgKtOamI+s0o52k41BUQHziltXkI5aILynW+ow2bsoD3Y6mpzmNZ0mrq9lo
hnyhbh/pXfRvvTX2MA3ZqzTPq6YI2oF3L+gjtoXscJyxWSyNo2jr6VgEjC8CDEtg
+MEN1j+mmopqtL1ScqN6KfQbfRyOL3xvCA0vYdMK0aMYs1aFcm/WckaBl6tGkIs9
W7/fXugK7ccvQX5S/MByFwPla+vLLqjitO3bM10WPezYnswky41i5HSiHpT4AboJ
daT5mYsxylSqHzLw88atHuKfebjPdYwon4R5XOYJvyr0UvBfuqoNKOa8WsVlz7/x
bsmy87iW3oTOZraT0zXpDiVwBjpAN7H9ShWLlFDvE+aW6hhYFmrij+9OUupLBY5F
S3DIkAra2w5iFW5xSYVX+xR8G5tIxqF/NTDHrh/sG4HTRdXdg83j3AASCliLgvBD
FO63foEox3ZgUTOttyOTAMLXLwbFObnrEC/KRtRYrbmxmioBnc0NXavDAZDdmxVN
1w75uqNgbe3PuBACLanARecCAwEAAQ==
-----END PUBLIC KEY-----
)" /* end public_key */
    },
    version_{
/* begin version */ R"(dev)" /* end version */
    },
    url_{
/* begin url */ R"(https://czkh6wsjq6.execute-api.ap-south-1.amazonaws.com/lice_gen/license_genrator)" /* end url */
    },
    lic_file_name_{"license_"},
    cred_file_name_{"credentials_"},
    dir_name_{"/.mowito"} {
  // get the home dir of current user
  char *home;
  if ((home = getenv("HOME")) == nullptr) {
    home = getpwuid(getuid())->pw_dir;
  }
  full_dir_ = std::string(home) + dir_name_;
  //std::cout<<full_dir<<std::endl;
}

std::vector<std::string> License::getMacAddresses() {
  const char *directory_name = "/sys/class/net/";
  struct dirent *dirent;
  DIR *dir;
  std::vector<std::string> mac_addresses;

  dir = opendir(directory_name);
  if (dir == nullptr) {
    printf("[E] Cannot open directory '%s'\n", directory_name);
    return mac_addresses;
  }
  while ((dirent = readdir(dir)) != nullptr) {
    // printf ("[V] Dir Name: %s\n", dirent->d_name);
    if (strcmp(dirent->d_name, ".") != 0 && strcmp(dirent->d_name, "..") != 0) {
      char file_name[100];
      strcpy(file_name, directory_name);
      strcat(file_name, dirent->d_name);
      strcat(file_name, "/address");
      std::vector<char> v;
      // printf ("[V] File Name: %s\n", file_name);
      if (FILE *file = fopen(file_name, "r")) {
        char buf[1024];
        while (size_t len = fread(buf, 1, sizeof(buf), file)) {
          v.insert(v.end(), buf, buf + len);
        }
        fclose(file);
        std::string str(v.begin(), v.end());
        str.erase(std::remove(str.begin(), str.end(), '\n'), str.end());
        if (str == "00:00:00:00:00:00")
          continue;
        mac_addresses.push_back(str);
      }
    }
  }
  closedir(dir);

  return mac_addresses;
}

bool License::readFromFile(std::string &contents, const std::string &path) {
  std::ifstream is{path, std::ifstream::binary};

  if (is) {
    // get length of file:
    is.seekg(0, std::ifstream::end);
    auto length = is.tellg();
    is.seekg(0, std::ifstream::beg);
    contents.resize(length);

    is.read(&contents[0], length);
    if (!is) {
      is.close();
      return false;
    }
  } else {
    return false;
  }

  is.close();
  return true;
}

bool License::doesPathExist(const std::string &s) {
  struct stat buffer{};
  return (stat(s.c_str(), &buffer) == 0);
}

bool License::getCredentials() {
  std::string credentials_path = full_dir_ + "/" + cred_file_name_;
  if (!doesPathExist(credentials_path)) {
    return false;
  } else {
    std::ifstream cred_file(credentials_path, std::ofstream::in);
    cred_file >> email_ >> username_ >> password_ >> robot_name_;
    return true;
  }
}

cpr::Response License::requestLicense(const std::string &mac_address) {
  return cpr::Get(cpr::Url{url_},
                  cpr::Header{{"Content-Type", "application/json"}},
                  cpr::Body{"{"
                            "\"user_email\": \"" + email_ + "\", "
                                                            "\"user_name\": \"" + username_ + "\", "
                                                                                              "\"password\": \""
                                + password_ + "\", "
                                              "\"mac_address\": \"" + mac_address + "\", "
                                                                                    "\"robot_name\": \"" + robot_name_
                                + "\", "
                                  "\"software_version\": \"" + version_ + "\""
                                                                          "}"
                  }
  );
}

bool License::getLicense() {
  if (!doesPathExist(full_dir_)) {
    if (mkdir(full_dir_.c_str(), S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH)) {
      return false;
    }
  }

  auto mac_addresses = getMacAddresses();

  if (mac_addresses.empty()) {
    return false;
  }

  int mac_number = 0;
  auto r = requestLicense(mac_addresses[mac_number]);
  int retries = 3;

  while (r.status_code != 200 && retries--) {
    switch (r.status_code) {
      case 0: ROS_ERROR("Could not establish connection to server. Please check your internet connection");
        break;
      case 400:ROS_ERROR("The software version doesn't seem correct please contact supplier.");
        return false;
      case 401:ROS_ERROR("The login credentials are incorrect.");
        return false;
      case 403:ROS_ERROR("Your account doesn't seem to have any licenses associated with it.");
        return false;
      case 411:ROS_ERROR("This robot has not been added to the database. Please register robot first.");
        return false;
      case 412:mac_number++;
        if (mac_number >= mac_addresses.size()) {
          ROS_ERROR("Robot name was registered with different mac address. Please register this bot with a new name.");
          return false;
        } else {
          retries++;
        }
        break;
    }
    r = requestLicense(mac_addresses[mac_number]);
  }
  if (r.status_code == 200) {
    std::string license_path = full_dir_ + "/" + lic_file_name_;
    std::ofstream lic_file(license_path, std::ofstream::binary);
    int first_qoute = r.text.find("\"");
    int second_qoute = r.text.find("\"", first_qoute + 1);
    lic_file << r.text.substr(first_qoute + 1, second_qoute - first_qoute - 1);
    lic_file.close();
    return true;
  } else {
    ROS_ERROR("Cannot download license. Code: %d", r.status_code);
    return false;
  }
}

bool License::verifyToken() {
  std::string license_path = full_dir_ + "/" + lic_file_name_;
  std::string license;
  if (!readFromFile(license, license_path)) {
    ROS_FATAL("Could not read license file.");
    return false;
  }

  try {
    auto decoded = jwt::decode(license);
    auto mac_addresses = getMacAddresses();

    for (auto &mac_address : mac_addresses) {
      auto verifier = jwt::verify()
          .allow_algorithm(jwt::algorithm::rs256{pub_key_, "", "", ""})
          .with_id(mac_address)
          .with_issuer(username_);

      try {
        verifier.verify(decoded);
        return true;
      } catch (const jwt::rsa_exception &e) {
        ROS_ERROR("License verification error. Code:001");
        return false;
      } catch (const jwt::signature_verification_exception &e) {
        ROS_ERROR("License file doesn't match with software version.");
        return false;
      } catch (const jwt::token_verification_exception &e) {
        if (strcmp(e.what(), "token verification failed: token expired") == 0) {
          ROS_ERROR("License has expired.");
          return false;
        } else if (strcmp(e.what(), "token verification failed: claim jti does not match expected") == 0) {
          continue;
        } else if (strcmp(e.what(), "token verification failed: claim iss does not match expected") == 0) {
          ROS_ERROR("License was issued to a different user.");
          return false;
        } else {
          ROS_ERROR("License verification error. Code:002.");
          return false;
        }
      } catch (...) {
        ROS_FATAL("License verification error. Code:003");
        return false;
      }
    }
  } catch (const std::invalid_argument &e) {
    ROS_FATAL("License verification error. Code:004");
    return false;
  } catch (const std::runtime_error &e) {
    ROS_FATAL("License verification error. Code:005");
    return false;
  } catch (...) {
    ROS_FATAL("License verification error. Code:006");
    return false;
  }
  ROS_ERROR("None of the mac addresses matched.");
  return false;
}

bool License::verifyLicense() {
  if (getCredentials()) {
    if (verifyToken()) {
      return true;
    }
    ROS_INFO("Getting License");
    if (getLicense()) {
      return verifyToken();
    } else {
      return false;
    }
  } else {
    ROS_ERROR("Could not get credentials. Please run registration.py");
    return false;
  }
}
