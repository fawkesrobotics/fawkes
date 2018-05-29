#*****************************************************************************
#               Makefile Build System for Fawkes: ROS Plugin
#                            -------------------
#   Created on Thu May 05 18:23:31 2011
#   Copyright (C) 2006-2012 by Tim Niemueller, AllemaniACs RoboCup Team
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
include $(BUILDSYSDIR)/ros.mk
include $(BUILDSYSDIR)/pcl.mk
include $(BUILDSYSDIR)/boost.mk
include $(BUILDCONFDIR)/tf/tf.mk

LIBS_ros = fawkescore fawkesutils fawkesaspects fawkesblackboard \
           fawkesinterface fawkesrosaspect
OBJS_ros = ros_plugin.o node_thread.o

LIBS_ros_talkerpub = fawkescore fawkesutils fawkesaspects fawkesblackboard \
		     fawkesinterface fawkesrosaspect
OBJS_ros_talkerpub = talkerpub_plugin.o talkerpub_thread.o

LIBS_ros_tf = fawkescore fawkesutils fawkesaspects fawkesblackboard \
	      fawkesinterface fawkesrosaspect fawkestf TransformInterface
OBJS_ros_tf = tf_plugin.o tf_thread.o

LIBS_ros_pcl = fawkescore fawkesutils fawkesaspects fawkesblackboard \
	      fawkesinterface fawkesrosaspect fawkestf fawkespcl_utils
OBJS_ros_pcl = pcl_plugin.o pcl_thread.o

LIBS_ros_images = fawkescore fawkesutils fawkesaspects fawkesblackboard \
	          fawkesinterface fawkesrosaspect fvutils
OBJS_ros_images = image_plugin.o image_thread.o

LIBS_ros_laserscan = m fawkescore fawkesutils fawkesaspects fawkesblackboard \
	          fawkesinterface fawkesrosaspect fvutils Laser360Interface \
		  Laser720Interface Laser1080Interface
OBJS_ros_laserscan = laserscan_plugin.o laserscan_thread.o

LIBS_ros_odometry = m fawkescore fawkesutils fawkesaspects fawkesblackboard \
		    fawkesinterface fawkestf fawkesrosaspect MotorInterface
OBJS_ros_odometry = odometry_plugin.o odometry_thread.o

LIBS_ros_cmdvel = m fawkescore fawkesutils fawkesaspects fawkesblackboard \
		  fawkesinterface fawkesrosaspect MotorInterface
OBJS_ros_cmdvel = cmdvel_thread.o cmdvel_plugin.o

LIBS_ros_navigator = m pthread fawkescore fawkesutils fawkesaspects fawkesblackboard \
		     fawkesinterface fawkesrosaspect NavigatorInterface
OBJS_ros_navigator = navigator_plugin.o navigator_thread.o

LIBS_ros_dynamic_reconfigure = m pthread fawkescore fawkesutils fawkesaspects fawkesblackboard \
		     fawkesinterface fawkesrosaspect DynamicReconfigureInterface
OBJS_ros_dynamic_reconfigure = dynamic_reconfigure_plugin.o dynamic_reconfigure_thread.o

LIBS_ros_navgraph_breakout = m pthread fawkescore fawkesutils fawkesaspects fawkesblackboard \
		     fawkesinterface fawkesrosaspect NavigatorInterface
OBJS_ros_navgraph_breakout = navgraph_breakout_plugin.o navgraph_breakout_thread.o

LIBS_ros_joint = fawkescore fawkesutils fawkesaspects fawkesblackboard \
	      fawkesinterface fawkesrosaspect JointInterface
OBJS_ros_joint = joint_plugin.o joint_thread.o

LIBS_ros_robot_description = fawkescore fawkesutils fawkesaspects fawkesrosaspect
OBJS_ros_robot_description = robot_description_plugin.o robot_description_thread.o

LIBS_ros_move_base = fawkescore fawkesutils fawkesaspects fawkesblackboard \
	      fawkesinterface fawkestf fawkesrosaspect NavigatorInterface
OBJS_ros_move_base = move_base_plugin.o move_base_thread.o

LIBS_ros_skiller = fawkescore fawkesutils fawkesaspects fawkesblackboard \
	      fawkesinterface fawkesrosaspect SkillerInterface
OBJS_ros_skiller = skiller_plugin.o skiller_thread.o

LIBS_ros_position_3d = fawkescore fawkesutils fawkesaspects fawkesblackboard \
	      fawkesinterface fawkesrosaspect Position3DInterface
OBJS_ros_position_3d = position_3d_thread.o position_3d_plugin.o

LIBS_ros_clock = fawkescore fawkesutils fawkesaspects fawkesblackboard \
	      fawkesinterface fawkesrosaspect
OBJS_ros_clock = clock_plugin.o clock_thread.o

LIBS_ros_imu = fawkescore fawkesutils fawkesaspects fawkesblackboard \
	      fawkesinterface fawkesrosaspect IMUInterface
OBJS_ros_imu = imu_plugin.o imu_thread.o

OBJS_all = $(OBJS_ros) $(OBJS_ros_talkerpub) $(OBJS_ros_tf) \
	   $(OBJS_ros_pcl) $(OBJS_ros_images) $(OBJS_ros_laserscan) $(OBJS_ros_odometry) \
	   $(OBJS_ros_cmdvel) $(OBJS_ros_navigator) $(OBJS_ros_joint) \
	   $(OBJS_ros_robot_description) $(OBJS_ros_move_base) \
	   $(OBJS_ros_skiller) $(OBJS_ros_clock) $(OBJS_ros_position_3d) \
	   $(OBJS_ros_dynamic_reconfigure) $(OBJS_ros_imu) $(OBJS_ros_navgraph_breakout)

ifneq ($(PKGCONFIG),)
  HAVE_TINYXML = $(if $(shell $(PKGCONFIG) --exists 'tinyxml'; echo $${?/1/}),1,0)
endif

ifeq ($(HAVE_ROS),1)
  PRESUBDIRS += aspect

  CFLAGS  += $(CFLAGS_ROS) -Wno-unknown-pragmas -Wno-deprecated-declarations
  LDFLAGS += $(LDFLAGS_ROS)

  CFLAGS_talkerpub_thread += $(call ros-pkg-cflags,std_msgs) $(CFLAGS)
  LDFLAGS_ros_talkerpub += $(LDFLAGS) $(call ros-pkg-lflags,std_msgs)

  PLUGINS_all = $(PLUGINDIR)/ros.so \
		$(PLUGINDIR)/ros-talkerpub.so

  ifeq ($(call ros-have-pkg,tf),1)
    ifeq ($(HAVE_TF),1)
      CFLAGS_tf_thread += $(CFLAGS) $(CFLAGS_TF) \
			  $(call ros-pkg-cflags,tf) -Wno-strict-aliasing
      CFLAGS_tf_plugin = $(CFLAGS_tf_thread)
      LDFLAGS_ros_tf += $(LDFLAGS_TF)  $(patsubst %bullet/lib,,$(call ros-pkg-lflags,tf))
      PLUGINS_all += $(PLUGINDIR)/ros-tf.so

      ifeq ($(call ros-have-pkg,tf2_msgs),1)
        CFLAGS_tf_thread += -DHAVE_TF2_MSGS $(call ros-pkg-cflags,tf2_msgs)
      else
        WARN_TARGETS += warning_tf2_msgs
      endif
      CFLAGS_tf_plugin = $(CFLAGS_tf_thread)
    else
      WARN_TARGETS += warning_fawkestf
    endif
  else
    WARN_TARGETS += warning_rostf
  endif

  ifeq ($(call ros-have-pkg,sensor_msgs),1)
    ifeq ($(HAVE_PCL),1)
      ifeq ($(call ros-have-pkg,pcl_conversions),1)
        CFLAGS_pcl_adapter += $(CFLAGS_PCL) $(CFLAGS_TF) $(CFLAGS)
        CFLAGS_pcl_thread += $(call ros-pkg-cflags,sensor_msgs) \
			     $(CFLAGS_TF) $(CFLAGS_PCL) $(CFLAGS)
        CFLAGS_pcl_plugin = $(CFLAGS_pcl_thread) $(CFLAGS)
        LDFLAGS_ros_pcl += $(LDFLAGS_PCL) $(LDFLAGS_TF) $(call ros-pkg-lflags,sensor_msgs)
        PLUGINS_all += $(PLUGINDIR)/ros-pcl.so
      else
        WARN_TARGETS += warning_pcl_conversions
      endif
    else
      WARN_TARGETS += warning_pcl
    endif

    ifeq ($(call ros-have-pkg,image_transport),1)
      ifeq ($(call ros-have-pkg,pcl_conversions),1)
        ifeq ($(HAVE_TINYXML),1)
          CFLAGS_image_thread += $(call ros-pkg-cflags,image_transport) $(CFLAGS)
          CFLAGS_image_plugin = $(CFLAGS_image_thread)
          LDFLAGS_ros_images += $(call ros-pkg-lflags,image_transport)
          PLUGINS_all += $(PLUGINDIR)/ros-images.so
        else
          WARN_TARGETS += warning_tinyxml
        endif
      else
        WARN_TARGETS += warning_images_pcl_conversions
      endif
    else
      WARN_TARGETS += warning_images
    endif

    CFLAGS_laserscan_thread += $(call ros-pkg-cflags,sensor_msgs) $(CFLAGS)
    CFLAGS_laserscan_plugin = $(CFLAGS_laserscan_thread) $(CFLAGS)
    LDFLAGS_ros_laserscan += $(call ros-pkg-lflags,sensor_msgs)
    PLUGINS_all += $(PLUGINDIR)/ros-laserscan.so

    CFLAGS_imu_thread += $(call ros-pkg-cflags,sensor_msgs) $(CFLAGS)
    CFLAGS_imu_plugin = $(CFLAGS_imu_thread)
    LDFLAGS_ros_imu += $(call ros-pkg-lflags,sensor_msgs)
    PLUGINS_all += $(PLUGINDIR)/ros-imu.so
  else
    WARN_TARGETS += warning_rospcl warning_images warning_laserscan warning_imu
  endif

  ifeq ($(call ros-have-pkg,nav_msgs),1)
    ifeq ($(HAVE_TF),1)
      CFLAGS_odometry_thread += $(call ros-pkg-cflags,nav_msgs) $(CFLAGS) $(CFLAGS_TF)
      CFLAGS_odometry_plugin = $(CFLAGS_odometry_thread)
      LDFLAGS_ros_odometry += $(call ros-pkg-lflags,nav_msgs) $(LDFLAGS_TF)
      PLUGINS_all += $(PLUGINDIR)/ros-odometry.so
    else
      WARN_TARGETS += warning_odometry
    endif
  else
    WARN_TARGETS += warning_nav_msgs
  endif

  ifeq ($(call ros-have-pkg,geometry_msgs),1)
    CFLAGS_cmdvel_thread += $(call ros-pkg-cflags,geometry_msgs) $(CFLAGS) $(CFLAGS_TF)
    CFLAGS_cmdvel_plugin = $(CFLAGS_cmdvel_thread)
    LDFLAGS_ros_cmdvel += $(call ros-pkg-lflags,geometry_msgs) $(LDFLAGS_TF)
    PLUGINS_all += $(PLUGINDIR)/ros-cmdvel.so
  else
    WARN_TARGETS += warning_geometry_msgs
  endif

  ifeq ($(call ros-have-pkg,dynamic_reconfigure),1)
    CFLAGS_dynamic_reconfigure_thread += $(call boost-lib-cflags,thread) \
                                         $(call ros-pkg-cflags,dynamic_reconfigure) \
                                         $(CFLAGS) $(CFLAGS_TF)
    CFLAGS_dynamic_reconfigure_plugin = $(CFLAGS_dynamic_reconfigure_thread)
    LDFLAGS_ros_dynamic_reconfigure +=  $(call boost-lib-ldflags,thread) \
                                        $(call boost-lib-ldflags,system) \
                                        $(call ros-pkg-lflags,dynamic_reconfigure) \
                                        $(LDFLAGS_TF)
    PLUGINS_all += $(PLUGINDIR)/ros-dynamic_reconfigure.so
  else
    WARN_TARGETS += warning_dynamic_reconfigure
  endif

  ifeq ($(call ros-have-pkgs,move_base_msgs dynamic_reconfigure actionlib),1)
    ifeq ($(HAVE_TF),1)
      CFLAGS_navigator_thread += $(call ros-pkgs-cflags,move_base_msgs dynamic_reconfigure actionlib) \
                                 $(call boost-lib-cflags,thread) \
                                 $(CFLAGS) $(CFLAGS_TF)
      CFLAGS_navigator_plugin = $(CFLAGS_navigator_thread)
      LDFLAGS_ros_navigator += $(call ros-pkgs-lflags,move_base_msgs dynamic_reconfigure actionlib) \
                               $(call boost-libs-ldflags,thread system) \
                               $(LDFLAGS_TF)

      CFLAGS_move_base_thread += $(call ros-pkgs-cflags,move_base_msgs actionlib) \
                                 $(call boost-libs-cflags,thread system) \
                                 $(CFLAGS) $(CFLAGS_TF)
      CFLAGS_move_base_plugin = $(CFLAGS_move_base_thread)
      LDFLAGS_ros_move_base += $(call ros-pkgs-lflags,move_base_msgs actionlib) \
                               $(call boost-libs-ldflags,thread system) -lpthread \
                               $(LDFLAGS_TF)

      PLUGINS_all += $(PLUGINDIR)/ros-navigator.so $(PLUGINDIR)/ros-move-base.so
    else
      WARN_TARGETS += warning_navigator_tf
    endif
  else
    WARN_TARGETS += $(addprefix warning_,$(call ros-missing-pkgs move_base_msgs dynamic_reconfigure actionlib))
  endif
  ifeq ($(call ros-have-pkg,sensor_msgs),1)
    CFLAGS_joint_thread = $(CFLAGS) $(call ros-pkg-cflags,sensor_msgs)
    CFLAGS_joint_plugin = $(CFLAGS_joint_thread)
    LDFLAGS_ros_joint += $(call ros-pkg-lflags,sensor_msgs)
    PLUGINS_all += $(PLUGINDIR)/ros-joint.so
  else
    WARN_TARGETS += warning_sensor_msgs
  endif
  ifeq ($(call ros-have-pkg,fawkes_msgs),1)
    CFLAGS_position_3d_thread += $(CFLAGS) $(call ros-pkg-cflags,fawkes_msgs)
    CFLAGS_position_3d_plugin = $(CFLAGS_position_3d_thread)
    LDFLAGS_ros_position_3d += $(call ros-pkg-lflags,fawkes_msgs)
    PLUGINS_all += $(PLUGINDIR)/ros-position-3d.so

    CFLAGS_navgraph_breakout_thread += $(CFLAGS) $(call ros-pkg-cflags,fawkes_msgs)
    CFLAGS_navgraph_breakout_plugin = $(CFLAGS_navgraph_breakout_thread)
    LDFLAGS_ros_navgraph_breakout += $(call ros-pkg-lflags,fawkes_msgs)
    PLUGINS_all += $(PLUGINDIR)/ros-navgraph-breakout.so
  else
    WARN_TARGETS += warning_fawkes_msgs
  endif

  ifeq ($(call ros-have-pkg,rosgraph_msgs),1)
    CFLAGS_clock_thread += $(call ros-pkg-cflags,rosgraph_msgs) $(CFLAGS)
    CFLAGS_clock_plugin = $(CFLAGS_clock_thread)
    LDFLAGS_ros_clock += $(LDFLAGS) $(call ros-pkg-lflags,rosgraph_msgs)
    PLUGINS_all += $(PLUGINDIR)/ros-clock.so
  else
    WARN_TARGETS += warning_rosgraph_msgs
  endif

  PLUGINS_all += $(PLUGINDIR)/ros-robot-description.so

  ifeq ($(call ros-have-pkg,fawkes_msgs),1)
    ifeq ($(call ros-have-pkg,actionlib),1)
      CFLAGS_skiller_thread += $(call ros-pkg-cflags,fawkes_msgs) \
      $(call ros-pkg-cflags,actionlib) \
      $(call boost-lib-cflags,thread) \
      $(call boost-lib-cflags,system) \
      $(CFLAGS)
      CFLAGS_skiller_plugin += $(CFLAGS_skiller_thread)
      LDFLAGS_ros_skiller += $(call ros-pkg-lflags,fawkes_msgs) \
      $(call ros-pkg-lflags,actionlib) \
      $(call boost-lib-ldflags,thread) \
      $(call boost-lib-ldflags,system) \
      $(LDFLAGS) -lpthread
      PLUGINS_all += $(PLUGINDIR)/ros-skiller.so
    else
      WARN_TARGETS += warning_actionlib      
    endif
  else
    WARN_TARGETS += warning_fawkes_msgs
  endif
else
  WARN_TARGETS += warning_ros
endif

ifeq ($(OBJSSUBMAKE),1)
all: $(WARN_TARGETS)

.PHONY: warning_ros warning_fawkestf warning_rostf warning_pcl warning_rospcl warning_tf2_msgs warning_dynamic_reconfigure
warning_ros:
	$(SILENT)echo -e "$(INDENT_PRINT)--> $(TRED)Omitting ROS Plugins$(TNORMAL) (ROS not found)"
warning_fawkestf:
	$(SILENT)echo -e "$(INDENT_PRINT)--> $(TRED)Omitting ROS tf plugin$(TNORMAL) (fawkestf not available)"
warning_rostf:
	$(SILENT)echo -e "$(INDENT_PRINT)--> $(TRED)Omitting ROS tf plugin$(TNORMAL) (ROS package tf not found)"
warning_tf2_msgs:
	$(SILENT)echo -e "$(INDENT_PRINT)--> $(TRED)Disabling tf2 support in tf plugin$(TNORMAL) (ROS package tf2_msgs not found)"
warning_pcl:
	$(SILENT)echo -e "$(INDENT_PRINT)--> $(TRED)Omitting ROS point clouds plugin$(TNORMAL) (pcl[-devel] not installed)"
warning_rospcl:
	$(SILENT)echo -e "$(INDENT_PRINT)--> $(TRED)Omitting ROS point clouds plugin$(TNORMAL) (ROS package pcl not found)"
warning_pcl_conversions:
	$(SILENT)echo -e "$(INDENT_PRINT)--> $(TRED)Omitting ROS point clouds plugin$(TNORMAL) (ROS package pcl_conversions not found)"
warning_images:
	$(SILENT)echo -e "$(INDENT_PRINT)--> $(TRED)Omitting ROS images plugin$(TNORMAL) (ROS package image_transport not found)"
warning_images_pcl_conversions:
	$(SILENT)echo -e "$(INDENT_PRINT)--> $(TRED)Omitting ROS images plugin$(TNORMAL) (ROS package pcl_conversions not found)"
warning_laserscan:
	$(SILENT)echo -e "$(INDENT_PRINT)--> $(TRED)Omitting ROS laser scan plugin$(TNORMAL) (ROS package sensor_msgs not found)"
warning_nav_msgs:
	$(SILENT)echo -e "$(INDENT_PRINT)--> $(TRED)Omitting ROS odometry Plugin$(TNORMAL) (ROS package nav_msgs not found)"
warning_odometry:
	$(SILENT)echo -e "$(INDENT_PRINT)--> $(TRED)Omitting ROS odometry Plugin$(TNORMAL) (Fawkes TF library not available)"
warning_geometry_msgs:
	$(SILENT)echo -e "$(INDENT_PRINT)--> $(TRED)Omitting ROS cmdvel Plugin$(TNORMAL) (ROS package geometry_msgs not found)"
warning_move_base_msgs:
	$(SILENT)echo -e "$(INDENT_PRINT)--> $(TRED)Omitting ROS navigator and move_base plugins$(TNORMAL) (ROS package move_base_msgs not found)"
warning_dynamic_reconfigure:
	$(SILENT)echo -e "$(INDENT_PRINT)--> $(TRED)Omitting ROS navigator and move_base plugins$(TNORMAL) (ROS package dynamic_reconfigure not found)"
warning_navigator_tf:
	$(SILENT)echo -e "$(INDENT_PRINT)--> $(TRED)Omitting ROS navigator and move_base plugins$(TNORMAL) (Fawkes TF library not available)"
warning_actionlib:
	$(SILENT)echo -e "$(INDENT_PRINT)--> $(TRED)Omitting ROS navigator, move_base and skiller plugins$(TNORMAL) (ROS package actionlib not found)"
warning_sensor_msgs:
	$(SILENT)echo -e "$(INDENT_PRINT)--> $(TRED)Omitting ROS JointState Plugin$(TNORMAL) (ROS package sensor_msgs not found)"
warning_rosgraph_msgs:
	$(SILENT)echo -e "$(INDENT_PRINT)--> $(TRED)Omitting ROS clock plugin$(TNORMAL) (ROS package rosgraph_msgs not found)"
warning_fawkes_msgs:
	$(SILENT)echo -e "$(INDENT_PRINT)--> $(TRED)Omitting ROS skiller plugin$(TNORMAL) (ROS package fawkes_msgs not found)"
	$(SILENT)echo -e "$(INDENT_PRINT)--> $(TRED)Omitting ROS Position 3D Plugin$(TNORMAL) (ROS package fawkes_msgs not found)"
	$(SILENT)echo -e "$(INDENT_PRINT)--> $(TRED)Omitting ROS Webview Plugin$(TNORMAL) (ROS package fawkes_msgs not found)"
warning_tinyxml:
	$(SILENT)echo -e "$(INDENT_PRINT)--> $(TRED)Omitting ROS images plugin$(TNORMAL) (tinyxml[-devel] not installed)"
warning_imu:
	$(SILENT)echo -e "$(INDENT_PRINT)--> $(TRED)Omitting ROS IMU plugin$(TNORMAL) (ROS package sensor_msgs not found)"
endif

include $(BUILDSYSDIR)/base.mk

