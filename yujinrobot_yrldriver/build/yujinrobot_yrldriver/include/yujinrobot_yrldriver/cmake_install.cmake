# Install script for directory: /home/iclab/test/src/driver_general/yujinrobot_yrldriver/yujinrobot_yrldriver/include/yujinrobot_yrldriver

# Set the install prefix
if(NOT DEFINED CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX "/home/iclab/test/src/driver_general/yujinrobot_yrldriver/build/lib_yujinrobot_yrldriver")
endif()
string(REGEX REPLACE "/$" "" CMAKE_INSTALL_PREFIX "${CMAKE_INSTALL_PREFIX}")

# Set the install configuration name.
if(NOT DEFINED CMAKE_INSTALL_CONFIG_NAME)
  if(BUILD_TYPE)
    string(REGEX REPLACE "^[^A-Za-z0-9_]+" ""
           CMAKE_INSTALL_CONFIG_NAME "${BUILD_TYPE}")
  else()
    set(CMAKE_INSTALL_CONFIG_NAME "")
  endif()
  message(STATUS "Install configuration: \"${CMAKE_INSTALL_CONFIG_NAME}\"")
endif()

# Set the component getting installed.
if(NOT CMAKE_INSTALL_COMPONENT)
  if(COMPONENT)
    message(STATUS "Install component: \"${COMPONENT}\"")
    set(CMAKE_INSTALL_COMPONENT "${COMPONENT}")
  else()
    set(CMAKE_INSTALL_COMPONENT)
  endif()
endif()

# Install shared libraries without execute permission?
if(NOT DEFINED CMAKE_INSTALL_SO_NO_EXE)
  set(CMAKE_INSTALL_SO_NO_EXE "1")
endif()

# Is this installation the result of a crosscompile?
if(NOT DEFINED CMAKE_CROSSCOMPILING)
  set(CMAKE_CROSSCOMPILING "FALSE")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/yujinrobot_yrldriver" TYPE FILE FILES
    "/home/iclab/test/src/driver_general/yujinrobot_yrldriver/yujinrobot_yrldriver/include/yujinrobot_yrldriver/NetworkInterface.hpp"
    "/home/iclab/test/src/driver_general/yujinrobot_yrldriver/yujinrobot_yrldriver/include/yujinrobot_yrldriver/YRLLOG.hpp"
    "/home/iclab/test/src/driver_general/yujinrobot_yrldriver/yujinrobot_yrldriver/include/yujinrobot_yrldriver/YRLSocket.hpp"
    "/home/iclab/test/src/driver_general/yujinrobot_yrldriver/yujinrobot_yrldriver/include/yujinrobot_yrldriver/YRLTCPSocket.hpp"
    "/home/iclab/test/src/driver_general/yujinrobot_yrldriver/yujinrobot_yrldriver/include/yujinrobot_yrldriver/YRLUDPSocket.hpp"
    "/home/iclab/test/src/driver_general/yujinrobot_yrldriver/yujinrobot_yrldriver/include/yujinrobot_yrldriver/extracode.hpp"
    "/home/iclab/test/src/driver_general/yujinrobot_yrldriver/yujinrobot_yrldriver/include/yujinrobot_yrldriver/yujinrobot_yrldriver.hpp"
    )
endif()

