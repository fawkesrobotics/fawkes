
#*****************************************************************************
#            Makefile Build System for Fawkes: Robotino Plugin
#                            -------------------
#   Created on Sun Nov 13 15:19:32 2011
#   Copyright (C) 2011 by Tim Niemueller, AllemaniACs RoboCup Team
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
include $(BUILDSYSDIR)/pcl.mk
include $(BUILDSYSDIR)/ros.mk
include $(BUILDCONFDIR)/tf/tf.mk

PRESUBDIRS = interfaces

# Uncomment to enable simple time tracking
#CFLAGS += -DUSE_TIMETRACKER

LIBS_robotino = m fawkescore fawkesutils fawkesaspects fawkesbaseapp \
		fawkesblackboard fawkesinterface MotorInterface GripperInterface \
		BatteryInterface RobotinoSensorInterface IMUInterface
OBJS_robotino = robotino_plugin.o com_thread.o sensor_thread.o act_thread.o

LIBS_robotino_ir_pcl = m fawkescore fawkesutils fawkesaspects fawkestf \
			 fawkesblackboard fawkesinterface fawkespcl_utils \
			 RobotinoSensorInterface
OBJS_robotino_ir_pcl = robotino_ir_pcl_plugin.o ir_pcl_thread.o

LIBS_robotino_ros_joints = m fawkescore fawkesutils fawkesaspects \
			   fawkesblackboard fawkesinterface fawkesrosaspect \
			   RobotinoSensorInterface
OBJS_robotino_ros_joints = robotino_ros_joints_plugin.o ros_joints_thread.o


ifeq ($(HAVE_OPENROBOTINO),1)
  CFLAGS  += -DHAVE_OPENROBOTINO $(CFLAGS_OPENROBOTINO)
  LDFLAGS_robotino += $(LDFLAGS_OPENROBOTINO)

	OBJS_robotino += openrobotino_com_thread.o
else
  WARN_TARGETS += warning_openrobotino
endif

ifeq ($(HAVE_LIBUDEV)$(HAVE_ROBOTINO_DIRECT_BOOST_LIBS)$(call boost-version-atleast,1,48,0),111)
	HAVE_ROBOTINO_DIRECT = 1

  CFLAGS += -DHAVE_ROBOTINO_DIRECT $(CFLAGS_CPP11) $(CFLAGS_LIBUDEV) $(CFLAGS_ROBOTINO_DIRECT_BOOST)
  LDFLAGS_robotino += $(LDFLAGS_LIBUDEV) $(LDFLAGS_ROBOTINO_DIRECT_BOOST)

	OBJS_robotino += direct_com_thread.o direct_com_message.o
else
  ifneq ($(HAVE_LIBUDEV),1)
    WARN_TARGETS += warning_libudev
  endif
  ifneq ($(HAVE_ROBOTINO_DIRECT_BOOST_LIBS),1)
    WARN_TARGETS_ROBOTINO_DIRECT_BOOST = $(foreach l,$(ROBOTINO_DIRECT_REQ_BOOST_LIBS),$(if $(call boost-have-lib,$l),, warning_robotino_direct_boost_$l))
	  WARN_TARGETS += $(WARN_TARGETS_ROBOTINO_DIRECT_BOOST)
  endif
  ifneq ($(call boost-version-atleast,1,48,0),1)
    WARN_TARGETS += warning_old_boost
  endif
endif

ifneq ($(HAVE_OPENROBOTINO)$(HAVE_ROBOTINO_DIRECT),)
  ifeq ($(HAVE_CPP11),1)

    CFLAGS += $(CFLAGS_CPP11)

    CFLAGS_sensor_thread = $(CFLAGS) -Wno-reorder -Wno-unused-function

    ifeq ($(HAVE_TF),1)
      CFLAGS_robotino_plugin  = $(CFLAGS) $(CFLAGS_TF)
      CFLAGS_act_thread       = $(CFLAGS_robotino_plugin)
      CFLAGS_sensor_thread   += $(CFLAGS) $(CFLAGS_TF)
      CFLAGS_openrobotino_com_thread      += $(CFLAGS) $(CFLAGS_TF)
      CFLAGS_direct_com_thread      += $(CFLAGS) $(CFLAGS_TF)
      LDFLAGS_robotino += $(LDFLAGS_TF)
      LIBS_robotino    += fawkestf
    endif

    PLUGINS_all = $(PLUGINDIR)/robotino.so

    ifeq ($(HAVE_PCL),1)
      CFLAGS_robotino_ir_pcl_plugin = $(CFLAGS) $(CFLAGS_PCL) $(CFLAGS_TF)
      CFLAGS_ir_pcl_thread = $(CFLAGS_robotino_ir_pcl_plugin)
      LDFLAGS_robotino_ir_pcl += $(LDFLAGS_PCL) $(LDFLAGS_TF)

      PLUGINS_all += $(PLUGINDIR)/robotino-ir-pcl.$(SOEXT)
    else
      WARN_TARGETS += warning_pcl
    endif
  else
    WARN_TARGETS += warning_cpp11
  endif
else
  WARN_TARGETS += warning_nodriver
endif

ifeq ($(HAVE_ROS),1)
  ifeq ($(call ros-have-pkg,sensor_msgs),1)
    CFLAGS_ros_joints_thread          = $(CFLAGS) $(CFLAGS_ROS) $(call ros-pkg-cflags,sensor_msgs)
    CFLAGS_robotino_ros_joints_plugin = $(CFLAGS_ros_joints_thread)
    LDFLAGS_robotino_ros_joints      += $(LDFLAGS_ROS) $(call ros-pkg-lflags,sensor_msgs)

    PLUGINS_all += $(PLUGINDIR)/robotino-ros-joints.$(SOEXT)
  else
    WARN_TARGETS += warning_sensor_msgs
  endif
else
  WARN_TARGETS += warning_ros
endif

OBJS_all = $(OBJS_robotino) $(OBJS_robotino_ir_pcl) \
	   $(OBJS_robotino_ros_joints)

ifeq ($(OBJSSUBMAKE),1)
  ifneq ($(WARN_TARGETS),)
all: $(WARN_TARGETS)
  endif
.PHONY: warning_openrobotino warning_pcl warning_ros warning_sensor_msgs $(WARN_TARGETS_ROBOTINO_DIRECT_BOOST) warning_nodriver warning_old_boost warning_cpp11
warning_openrobotino:
	$(SILENT)echo -e "$(INDENT_PRINT)--> $(TBROWN)Omitting OpenRobotino API support$(TNORMAL) (OpenRobotino API not found)"
warning_nodriver:
	$(SILENT)echo -e "$(INDENT_PRINT)--> $(TBROWN)Omitting Robotino platform support$(TNORMAL) (driver prerequisites not met)"
warning_pcl:
	$(SILENT)echo -e "$(INDENT_PRINT)--> $(TBROWN)Omitting Robotino PCL plugins$(TNORMAL) (PCL not found)"
warning_ros:
	$(SILENT)echo -e "$(INDENT_PRINT)--> $(TBROWN)Omitting Robotino Joints plugin$(TNORMAL) (ROS not found)"
warning_sensor_msgs:
	$(SILENT)echo -e "$(INDENT_PRINT)--> $(TBROWN)Omitting Robotino Joints plugin$(TNORMAL) (ROS package sensor_msgs not found)"
warning_libudev:
	$(SILENT)echo -e "$(INDENT_PRINT)--> $(TYELLOW)Omitting Robotino direct communication support$(TNORMAL) (libudev not available)"
$(WARN_TARGETS_ROBOTINO_DIRECT_BOOST): warning_robotino_direct_boost_%:
	$(SILENT)echo -e "$(INDENT_PRINT)--> $(TYELLOW)Omitting Robotino direct communication support$(TNORMAL) (Boost library $* not found)"
warning_old_boost:
	$(SILENT)echo -e "$(INDENT_PRINT)--> $(TYELLOW)Omitting Robotino direct communication support$(TNORMAL) (Boost library too old, require at least 1.0.48, have $(call boost-version-parse,$(BOOST_VERSION)))"
warning_cpp11:
	$(SILENT)echo -e "$(INDENT_PRINT)--> $(TYELLOW)Omitting Robotino direct communication support$(TNORMAL) (C++11 support required)"
endif

include $(BUILDSYSDIR)/base.mk
