# *****************************************************************************
# CMake Build System for Fawkes
# -------------------
# Copyright (C) 2023 by Tarik Viehmann and Daniel Swoboda
#
# *****************************************************************************
#
# This program is free software: you can redistribute it and/or modify it under
# the terms of the GNU General Public License as published by the Free Software
# Foundation, either version 3 of the License, or (at your option) any later
# version.
#
# This program is distributed in the hope that it will be useful, but WITHOUT
# ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS
# FOR A PARTICULAR PURPOSE.  See the GNU General Public License for more
# details.
#
# You should have received a copy of the GNU General Public License along with
# this program.  If not, see <http://www.gnu.org/licenses/>.
#
# *****************************************************************************
set(BUILD_WITH_ROS_2
    ON
    CACHE BOOL "Build with ROS 2")

# Fetch the ROS_DISTRO environment variable
if(DEFINED ENV{ROS_DISTRO})
    set(ROS_DISTRO $ENV{ROS_DISTRO})
else()
    set(BUILD_WITH_ROS_2 OFF CACHE BOOL "Build with ROS 2" FORCE)
    message(WARNING "ROS_DISTRO environment variable is not set. BUILD_WITH_ROS_2 is forced to OFF.")
endif()

# Construct the ROS_2_INSTALL_DIR path using the ROS_DISTRO variable if BUILD_WITH_ROS_2 is ON
if(BUILD_WITH_ROS_2)
    set(ROS_2_INSTALL_DIR "/usr/lib64/ros2-${ROS_DISTRO}" CACHE STRING "ROS 2 install directory")

    # Check if the constructed ROS_2_INSTALL_DIR exists
    if(NOT EXISTS "${ROS_2_INSTALL_DIR}")
        set(BUILD_WITH_ROS_2 OFF CACHE BOOL "Build with ROS 2" FORCE)
        message(WARNING "The directory of ROS_2_INSTALL_DIR ( ${ROS_2_INSTALL_DIR} ) does not exist. BUILD_WITH_ROS_2 is forced to OFF.")
    endif()
endif()

if(BUILD_WITH_ROS_2)
  find_package(ament_cmake QUIET)
  remember_dependency(ament_cmake)
  find_package(rclcpp QUIET)
  remember_dependency(rclcpp)
  find_package(rmw QUIET)
  remember_dependency(rmw)
  find_package(rosidl_typesupport_interface QUIET)
  remember_dependency(rosidl_typesupport_interface)
  find_package(rcl_interfaces QUIET)
  remember_dependency(rcl_interfaces)
  find_package(std_msgs QUIET)
  remember_dependency(std_msgs)
  find_package(geometry_msgs QUIET)
  remember_dependency(geometry_msgs)
  find_package(sensor_msgs QUIET)
  remember_dependency(sensor_msgs)
  find_package(console_bridge QUIET)
  remember_dependency(console_bridge)
  find_package(class_loader QUIET)
  remember_dependency(class_loader)
  find_package(tinyxml2 QUIET)
  remember_dependency(tinyxml2)
  find_package(unique_identifier_msgs QUIET)
  remember_dependency(unique_identifier_msgs)
  find_package(action_msgs QUIET)
  remember_dependency(action_msgs)
  if(ament_cmake_FOUND
     AND rclcpp_FOUND
     AND rmw_FOUND
     AND rosidl_typesupport_interface_FOUND)
    set(ROS_2_FOUND 1)
  endif()
endif()

function(depend_on_ros2 target)
  depend_on_ros2_libs(${target}
                      "rclcpp;rmw;rosidl_typesupport_interface;rcl_interfaces")
  target_compile_definitions(${target} PUBLIC HAVE_ROS2
                                              BOOST_BIND_GLOBAL_PLACEHOLDERS)
  target_compile_options(${target} PUBLIC -Wno-unknown-pragmas
                                          -Wno-deprecated-declarations)
endfunction()

function(depend_on_ros2_libs target libs)
  if(BUILD_WITH_ROS_2 AND ROS_2_FOUND)
    depend_on_find_package_libs(${target} "${libs}")
    foreach(lib ${libs})
      target_include_directories(
        ${target} PUBLIC "${ROS_2_INSTALL_DIR}/include/${${lib}_INCLUDE_DIRS}/")

      if("${lib}" STREQUAL "std_msgs")
        target_include_directories(${target}
                                   PUBLIC ${${lib}_INCLUDE_DIRS}/${lib})
      endif()
      if("${lib}" STREQUAL "image_transport")
        target_include_directories(${target}
                                   PUBLIC "${ROS_2_INSTALL_DIR}/include/${lib}")
      endif()
    endforeach()
  else()
    set_target_properties(${target} PROPERTIES EXCLUDE_FROM_ALL 1
                                               EXCLUDE_FROM_DEFAULT_BUILD 1)
    if(NOT BUILD_WITH_ROS_2)
      target_skipped_message(${target} "ROS 2 disabled (BUILD_WITH_ROS_2)")
    else()
      if(NOT ROS_1_FOUND)
        target_skipped_message(${target} "ros2-rclcpp[-devel]")
      endif()
    endif()
  endif()
endfunction()
