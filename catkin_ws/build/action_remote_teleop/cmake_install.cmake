# Install script for directory: /home/annawong/remote_teleop/catkin_ws/src/action_remote_teleop

# Set the install prefix
if(NOT DEFINED CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX "/home/annawong/remote_teleop/catkin_ws/install")
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
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/action_remote_teleop/action" TYPE FILE FILES "/home/annawong/remote_teleop/catkin_ws/src/action_remote_teleop/action/TurnInPlace.action")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/action_remote_teleop/msg" TYPE FILE FILES
    "/home/annawong/remote_teleop/catkin_ws/devel/share/action_remote_teleop/msg/TurnInPlaceAction.msg"
    "/home/annawong/remote_teleop/catkin_ws/devel/share/action_remote_teleop/msg/TurnInPlaceActionGoal.msg"
    "/home/annawong/remote_teleop/catkin_ws/devel/share/action_remote_teleop/msg/TurnInPlaceActionResult.msg"
    "/home/annawong/remote_teleop/catkin_ws/devel/share/action_remote_teleop/msg/TurnInPlaceActionFeedback.msg"
    "/home/annawong/remote_teleop/catkin_ws/devel/share/action_remote_teleop/msg/TurnInPlaceGoal.msg"
    "/home/annawong/remote_teleop/catkin_ws/devel/share/action_remote_teleop/msg/TurnInPlaceResult.msg"
    "/home/annawong/remote_teleop/catkin_ws/devel/share/action_remote_teleop/msg/TurnInPlaceFeedback.msg"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/action_remote_teleop/cmake" TYPE FILE FILES "/home/annawong/remote_teleop/catkin_ws/build/action_remote_teleop/catkin_generated/installspace/action_remote_teleop-msg-paths.cmake")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include" TYPE DIRECTORY FILES "/home/annawong/remote_teleop/catkin_ws/devel/include/action_remote_teleop")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/roseus/ros" TYPE DIRECTORY FILES "/home/annawong/remote_teleop/catkin_ws/devel/share/roseus/ros/action_remote_teleop")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/common-lisp/ros" TYPE DIRECTORY FILES "/home/annawong/remote_teleop/catkin_ws/devel/share/common-lisp/ros/action_remote_teleop")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/gennodejs/ros" TYPE DIRECTORY FILES "/home/annawong/remote_teleop/catkin_ws/devel/share/gennodejs/ros/action_remote_teleop")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  execute_process(COMMAND "/usr/bin/python3" -m compileall "/home/annawong/remote_teleop/catkin_ws/devel/lib/python3/dist-packages/action_remote_teleop")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/python3/dist-packages" TYPE DIRECTORY FILES "/home/annawong/remote_teleop/catkin_ws/devel/lib/python3/dist-packages/action_remote_teleop")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/pkgconfig" TYPE FILE FILES "/home/annawong/remote_teleop/catkin_ws/build/action_remote_teleop/catkin_generated/installspace/action_remote_teleop.pc")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/action_remote_teleop/cmake" TYPE FILE FILES "/home/annawong/remote_teleop/catkin_ws/build/action_remote_teleop/catkin_generated/installspace/action_remote_teleop-msg-extras.cmake")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/action_remote_teleop/cmake" TYPE FILE FILES
    "/home/annawong/remote_teleop/catkin_ws/build/action_remote_teleop/catkin_generated/installspace/action_remote_teleopConfig.cmake"
    "/home/annawong/remote_teleop/catkin_ws/build/action_remote_teleop/catkin_generated/installspace/action_remote_teleopConfig-version.cmake"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/action_remote_teleop" TYPE FILE FILES "/home/annawong/remote_teleop/catkin_ws/src/action_remote_teleop/package.xml")
endif()

