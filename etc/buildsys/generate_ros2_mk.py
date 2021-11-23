import subprocess
import os
import sys, getopt
import argparse
import shutil


def find(name, path):
    for root, dirs, files in os.walk(path):
        if name in files or name in dirs:
            return os.path.join(root, name)
    return ""

def main(argv):
    parser = argparse.ArgumentParser()
    parser.add_argument("--required_package_list", nargs="+", default=["rclcpp", "rclcpp_action", "std_msgs", "geometry_msgs", "tf2", "tf2_msgs", "rosgraph_msgs", "sensor_msgs", "image_transport", \
        "nav2_msgs", "nav_msgs", "pcl_conversions", "pcl_msgs", "fawkes_msgs", "visualization_msgs"])
    value = parser.parse_args(argv)
    required_package_list = value.required_package_list
    print(required_package_list)
    ros2_mk_path = find("ros2.mk", os.environ.get('FAWKES_DIR'))
    if not os.environ.get('FAWKES_DIR'):
        print("environment variable FAWKES_DIR not set")
        return
    if not os.environ.get('COLCON_PREFIX_PATH'):
        print("environment variable COLCON_PREFIX_PATH not set -> maybe source the ROS2 workspace")
        return
    setup_bashs = ','.join([setup + "/setup.bash" for setup in os.environ.get('COLCON_PREFIX_PATH').split(":")])
    

    get_flags_py_path = find("get_flags.py", os.environ.get('FAWKES_DIR'))
    if os.path.exists("/tmp/tmp_ws_for_flags"):
        shutil.rmtree("/tmp/tmp_ws_for_flags")
    print("path of ros2.mk:", ros2_mk_path)
    print("setup.bash's being sourced", setup_bashs)
    with open(ros2_mk_path, 'w') as f:
        mk = '''
#*****************************************************************************
#                 Makefile Build System for Fawkes: ROS2 bits
#                            -------------------
#   Created on Sunday November 07 12:00:00 2021
#   Copyright (C) 2021 by Gjorgji Nikolovski, Carologistics RoboCup Team
#
#*****************************************************************************
#
#   This program is free software; you can redistribute it and/or modify
#   it under the terms of the GNU General Public License as published by
#   the Free Software Foundation; either version 2 of the License, or
#   (at your option) any later version.
#
#*****************************************************************************



ifndef __buildsys_config_mk_
$(error config.mk must be included before ros2.mk)
endif

ifndef __buildsys_ros2_mk_
__buildsys_ros2_mk_ := 1

  # GENERATED STATIC FLAGS BEGIN'''
        for package in required_package_list:
            print(f'getting flags for package {package}...')
            cflags = ""
            cflags = subprocess.check_output(f'/usr/bin/python3 {get_flags_py_path} -s {setup_bashs} -p {package} -b -c', shell=True)[:-2].decode('ascii')
            lflags = subprocess.check_output(f'/usr/bin/python3 {get_flags_py_path} -s {setup_bashs} -p {package} -b -l', shell=True)[:-2].decode('ascii')
            mk += f'''
  {package.upper()}_cflags={cflags}
  {package.upper()}_lflags= {lflags}'''
        mk += f'''
  # GENERATED STATIC FLAGS END

  #ROS2 make utilities
  ros2-pkg-path = 
  ros2-have-pkgs    = $(if $(strip $(subst 1,,$(foreach p,$1,$(call ros2-have-pkg,$p)))),0,1)
  ros2-pkgs-cflags  = $(foreach p,$1,$(call ros2-pkg-cflags,$p) )
  ros2-pkgs-lflags  = $(foreach p,$1,$(call ros2-pkg-lflags,$p) )
  ros2-missing-pkgs = $(strip $(foreach p,$1,$(if $(subst 1,,$(call ros2-have-pkg,$p)),$p )))

  ros2-have-pkg     = $(if $(shell /usr/bin/python3 {get_flags_py_path} -s {setup_bashs} -p $(1) -e; echo $${{?/1/}}),1,0)
  ros2-pkg-cflags   = $(shell /usr/bin/python3 {get_flags_py_path} -s {setup_bashs} -p $(1) -b --cflags)
  ros2-pkg-lflags   = $(subst -l:,,$(shell /usr/bin/python3 {get_flags_py_path} -s {setup_bashs} -p $(1) --lflags))
  ros2-pkg-version  = $(shell /usr/bin/python3 {get_flags_py_path} -s {setup_bashs} -p $(1) -v)
  ros2-pkg-version-atleast = $(if $(shell /usr/bin/python3 {get_flags_py_path} -p $(1) -v; echo $${{?/1/}}),1,0)

  HAVE_ROS2 = $(call ros2-have-pkg,rclcpp)

  CFLAGS_ROS2  = -DBOOST_BIND_GLOBAL_PLACEHOLDERS $(RCLCPP_cflags)
  LDFLAGS_ROS2 = $(RCLCPP_lflags)

endif # __buildsys_ros2_mk_
'''
        print('writing ros2.mk...')
        f.write(mk)




if __name__=="__main__":
    main(sys.argv[1:])
