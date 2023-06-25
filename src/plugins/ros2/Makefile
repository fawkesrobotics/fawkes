#*****************************************************************************
#               Makefile Build System for Fawkes: ROS 2 Plugins
#                            -------------------
#   Created on Thu May 05 18:23:31 2011
#   Copyright (C) 2006-2012 by Gjorgji Nikolovski
#
#*****************************************************************************
#
#   This program is free software; you can redistribute it and/or modify
#   it under the terms of the GNU General Public License as published by
#   the Free Software Foundation; either version 2 of the License, or
#   (at your option) any later version.
#
#*****************************************************************************

BASEDIR = ../../..
include $(BASEDIR)/etc/buildsys/config.mk
include $(BUILDSYSDIR)/ros2.mk
include $(BUILDSYSDIR)/pcl.mk
include $(BUILDSYSDIR)/boost.mk
include $(BUILDCONFDIR)/tf/tf.mk

PRESUBDIRS = aspect

LIBS_ros2 = fawkescore fawkesutils fawkesaspects fawkesblackboard \
           fawkesinterface fawkesros2aspect
OBJS_ros2 = ros2_plugin.o node_thread.o

LIBS_ros2_cmdvel = m fawkescore fawkesutils fawkesaspects fawkesblackboard \
		  fawkesinterface fawkesros2aspect MotorInterface
OBJS_ros2_cmdvel = cmdvel_thread.o cmdvel_plugin.o

LIBS_ros2_talkerpub = fawkescore fawkesutils fawkesaspects fawkesblackboard \
		     fawkesinterface fawkesros2aspect
OBJS_ros2_talkerpub = talkerpub_plugin.o talkerpub_thread.o

LIBS_ros2_tf2 = fawkescore fawkesutils fawkesaspects fawkesblackboard \
	      fawkesinterface fawkesros2aspect fawkestf TransformInterface
OBJS_ros2_tf2 = tf2_plugin.o tf2_thread.o

LIBS_ros2_clock = fawkescore fawkesutils fawkesaspects fawkesblackboard \
	      fawkesinterface fawkesros2aspect
OBJS_ros2_clock = clock_plugin.o clock_thread.o

LIBS_ros2_images = fawkescore fawkesutils fawkesaspects fawkesblackboard \
	          fawkesinterface fawkesros2aspect fvutils
OBJS_ros2_images = image_plugin.o image_thread.o

LIBS_ros2_imu = fawkescore fawkesutils fawkesaspects fawkesblackboard \
	      fawkesinterface fawkesros2aspect IMUInterface
OBJS_ros2_imu = imu_plugin.o imu_thread.o

LIBS_ros2_joint = fawkescore fawkesutils fawkesaspects fawkesblackboard \
	      fawkesinterface fawkesros2aspect JointInterface
OBJS_ros2_joint = joint_plugin.o joint_thread.o

LIBS_ros2_odometry = m fawkescore fawkesutils fawkesaspects fawkesblackboard \
			fawkesinterface fawkestf fawkesros2aspect MotorInterface
OBJS_ros2_odometry = odometry_plugin.o odometry_thread.o

LIBS_ros2_navigator = m pthread fawkescore fawkesutils fawkesaspects fawkesblackboard \
			fawkesinterface fawkesros2aspect Position3DInterface \
			NavigatorInterface
OBJS_ros2_navigator = navigator_plugin.o navigator_thread.o

LIBS_ros2_robot_description = fawkescore fawkesutils fawkesaspects fawkesros2aspect
OBJS_ros2_robot_description = robot_description_plugin.o robot_description_thread.o

LIBS_ros2_position_3d = fawkescore fawkesutils fawkesaspects fawkesblackboard \
	      fawkesinterface fawkesros2aspect Position3DInterface
OBJS_ros2_position_3d = position_3d_thread.o position_3d_plugin.o

OBJS_ros2_laserscan = laserscan_thread.o laserscan_plugin.o
LIBS_ros2_laserscan = m fawkescore fawkesutils fawkesaspects fawkesblackboard \
			fawkesinterface fawkesros2aspect fvutils Laser360Interface \
			Laser720Interface Laser1080Interface

#LIBS_ros2_pcl = fawkescore fawkesutils fawkesaspects fawkesblackboard \
#	      fawkesinterface fawkesros2aspect fawkestf fawkespcl_utils
#OBJS_ros2_pcl = pcl_plugin.o pcl_thread.o

OBJS_ros2_laserscan = laserscan_plugin.o laserscan_thread.o

LIBS_ros2_pcl = fawkescore fawkesutils fawkesaspects fawkesblackboard \
		fawkesinterface fawkesros2aspect fawkestf fawkespcl_utils
OBJS_ros2_pcl = pcl_plugin.o pcl_thread.o

LIBS_ros2_skiller = fawkescore fawkesutils fawkesaspects fawkesblackboard \
			fawkesinterface fawkesros2aspect SkillerInterface
OBJS_ros2_skiller = skiller_plugin.o skiller_thread.o

OBJS_all = $(OBJS_ros2)  \
           $(OBJS_ros2_cmdvel) \
           $(OBJS_ros2_talkerpub) \
           $(OBJS_ros2_tf) \
           $(OBJS_ros2_clock) \
           $(OBJS_ros2_images) \
           $(OBJS_ros2_imu) \
           $(OBJS_ros2_joint) \
           $(OBJS_ros2_odometry) \
           $(OBJS_ros2_robot_description) \
           $(OBJS_ros2_position_3d) \
           $(OBJS_ros2_skiller) \
           $(OBJS_ros2_laserscan) \
           $(OBJS_ros2_navigator)

PLUGINS_all = $(PLUGINDIR)/ros2.so \
              $(PLUGINDIR)/ros2-cmdvel.so \
              $(PLUGINDIR)/ros2-talkerpub.so \
              $(PLUGINDIR)/ros2-tf2.so \
              $(PLUGINDIR)/ros2-clock.so \
              $(PLUGINDIR)/ros2-images.so \
              $(PLUGINDIR)/ros2-imu.so \
              $(PLUGINDIR)/ros2-joint.so \
              $(PLUGINDIR)/ros2-odometry.so \
              $(PLUGINDIR)/ros2-robot-description.so \
              $(PLUGINDIR)/ros2-position-3d.so \
              $(PLUGINDIR)/ros2-laserscan.so \
              $(PLUGINDIR)/ros2-navigator.so \
              $(PLUGINDIR)/ros2-pcl.so \
              $(PLUGINDIR)/ros2-skiller.so

CFLAGS  += $(CFLAGS_ROS2) -Wno-unknown-pragmas -Wno-deprecated-declarations
LDFLAGS += $(LDFLAGS_ROS2)

CFLAGS_cmdvel_thread += $(CFLAGS) $(call ros2-pkg-cflags,geometry_msgs)
CFLAGS_cmdvel_plugin = $(CFLAGS_cmdvel_thread)
LDFLAGS_ros2_cmdvel += $(LDFLAGS) $(call ros2-pkg-lflags,geometry_msgs)

CFLAGS_talkerpub_thread += $(CFLAGS) $(call ros2-pkg-cflags,std_msgs)
CFLAGS_talkerpub_plugin += $(CFLAGS_talkerpub_thread)
LDFLAGS_ros2_talkerpub += $(LDFLAGS) $(call ros2-pkg-lflags,std_msgs)

CFLAGS_tf2_thread += $(CFLAGS) $(CFLAGS_TF) $(call ros2-pkg-cflags,tf2) $(call ros2-pkg-cflags,tf2_msgs)
CFLAGS_tf2_plugin = $(CFLAGS_tf2_thread)
LDFLAGS_ros2_tf2 += $(LDFLAGS) $(call ros2-pkg-lflags,tf2) $(call ros2-pkg-lflags,tf2_msgs)


CFLAGS_clock_thread += $(CFLAGS) $(call ros2-pkg-cflags,rosgraph_msgs)
CFLAGS_clock_plugin = $(CFLAGS_clock_thread)
LDFLAGS_ros2_clock += $(LDFLAGS) $(call ros2-pkg-lflags,rosgraph_msgs)

CFLAGS_image_thread += $(CFLAGS) $(call ros2-pkg-cflags,image_transport) $(call ros2-pkg-cflags,sensor_msgs)
CFLAGS_image_plugin = $(CFLAGS_image_thread)
LDFLAGS_ros2_images += $(LDFLAGS) $(call ros2-pkg-lflags,image_transport) $(call ros2-pkg-lflags,sensor_msgs)

CFLAGS_imu_thread += $(CFLAGS) $(call ros2-pkg-cflags,sensor_msgs)
CFLAGS_imu_plugin = $(CFLAGS_imu_thread)
LDFLAGS_ros2_imu += $(LDFLAGS) $(call ros2-pkg-lflags,sensor_msgs)

CFLAGS_joint_thread = $(CFLAGS) $(call ros2-pkg-cflags,sensor_msgs)
CFLAGS_joint_plugin = $(CFLAGS_joint_thread)
LDFLAGS_ros2_joint += $(LDFLAGS) $(call ros2-pkg-lflags,sensor_msgs)

CFLAGS_odometry_thread += $(CFLAGS) $(call ros2-pkg-cflags,nav_msgs) $(CFLAGS_TF)
CFLAGS_odometry_plugin = $(CFLAGS_odometry_thread)
LDFLAGS_ros2_odometry += $(LDFLAGS) $(call ros2-pkg-lflags,nav_msgs) $(LDFLAGS_TF)

CFLAGS_robot_description_thread += $(CFLAGS)
CFLAGS_robot_description_plugin = $(CFLAGS_robot_description_thread)
LDFLAGS_ros2_robot_description += $(LDFLAGS)

CFLAGS_navigator_thread += $(CFLAGS) $(call ros2-pkg-cflags,nav2_msgs) $(call ros2-pkg-cflags,rclcpp_action) ${CFLAGS_TF}
CFLAGS_navigator_plugin = $(CFLAGS_navigator_thread)
LDFLAGS_ros2_navigator += $(LDFLAGS) $(call ros2-pkg-lflags,nav2_msgs) $(call ros2-pkg-lflags,rclcpp_action)

#CFLAGS_pcl_adapter += $(CFLAGS) $(CFLAGS_TF) $(CFLAGS_PCL) $(SENSOR_MSGS_cflags) $(PCL_CONVERSIONS_cflags)
#CFLAGS_pcl_thread += $(CFLAGS) $(CFLAGS_TF) $(CFLAGS_PCL) $(SENSOR_MSGS_cflags) $(PCL_CONVERSIONS_cflags)
#CFLAGS_pcl_plugin = $(CFLAGS_pcl_thread)
#LDFLAGS_ros2_pcl += $(LDFLAGS) $(LDFLAGS_TF) $(LDFLAGS_PCL) $(SENSOR_MSGS_lflags) $(PCL_CONVERSIONS_lflags)

CFLAGS_laserscan_thread += $(CFLAGS) $(call ros2-pkg-cflags,sensor_msgs)
CFLAGS_laserscan_plugin = $(CFLAGS_laserscan_thread)
LDFLAGS_ros2_laserscan += $(LDFLAGS) $(call ros2-pkg-lflags,sensor_msgs)

CFLAGS_position_3d_thread += $(CFLAGS) $(call ros2-pkg-cflags,fawkes_msgs)
CFLAGS_position_3d_plugin = $(CFLAGS_position_3d_thread)
LDFLAGS_ros_position_3d += $(LDFLAGS) $(call ros2-pkg-lflags,fawkes_msgs)

PLUGINS_build += $(PLUGINDIR)/ros2.so \
                 $(PLUGINDIR)/ros2-cmdvel.so \
                 $(PLUGINDIR)/ros2-talkerpub.so \
                 $(PLUGINDIR)/ros2-tf2.so \
                 $(PLUGINDIR)/ros2-clock.so \
                 $(PLUGINDIR)/ros2-images.so \
                 $(PLUGINDIR)/ros2-imu.so \
                 $(PLUGINDIR)/ros2-joint.so \
                 $(PLUGINDIR)/ros2-odometry.so \
                 $(PLUGINDIR)/ros2-robot-description.so \
                 $(PLUGINDIR)/ros2-laserscan.so \
                 $(PLUGINDIR)/ros2-navigator.so
#                 $(PLUGINDIR)/ros2-pcl.so
#                 #$(PLUGINDIR)/ros2-position-3d.so

include $(BUILDSYSDIR)/base.mk

