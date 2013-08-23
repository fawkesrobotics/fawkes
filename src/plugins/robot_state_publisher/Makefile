#*****************************************************************************
#      Makefile Build System for Fawkes: Robot State Publisher Plugin
#
#   Created on Thu Aug 22 11:14:00 2013
#   Copyright (C) 2013 by Till Hofmann, AllemaniACs RoboCup Team
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
include $(BUILDSYSDIR)/boost.mk
include $(BUILDCONFDIR)/tf/tf.mk

LIBS_robot_state_publisher = fawkescore fawkesutils fawkesaspects fawkesrosaspect fawkestf
OBJS_robot_state_publisher = robot_state_publisher_plugin.o robot_state_publisher_thread.o
OBJS_all = $(OBJS_robot_state_publisher)

ifeq ($(HAVE_ROS),1)
	ifeq ($(HAVE_TF),1)
		ifeq ($(call ros-have-pkg,sensor_msgs),1)
			ifeq ($(call ros-have-pkg,kdl),1)
				ifeq ($(call ros-have-pkg,kdl_parser),1)
					PLUGINS_all = $(PLUGINDIR)/robot_state_publisher.so
					CFLAGS += $(CFLAGS_ROS) $(call ros-pkg-cflags,sensor_msgs) \
						$(call ros-pkg-cflags,kdl) \
						$(call ros-pkg-cflags,kdl_parser) \
						$(call boost-lib-cflags,scoped_ptr) \
						$(CFLAGS_TF)
					LDFLAGS += $(LDFLAGS_ROS) $(call ros-pkg-lflags,sensor_msgs) \
						$(call ros-pkg-lflags,kdl) \
						$(call ros-pkg-lflags,kdl_parser) \
						$(call boost-lib-lflags,scoped_ptr) \
						$(LDFLAGS_TF)
				else
					WARN_TARGETS += warning_kdl_parser
				endif
			else
				WARN_TARGETS += warning_kdl
			endif
		else
			WARN_TARGETS += warning_sensor_msgs
		endif
	else
		WARN_TARGETS +=	warning_tf
	endif
else
	WARN_TARGETS += warning_ros
endif

ifeq ($(OBJSSUBMAKE),1)
all: $(WARN_TARGETS)

.PHONY: warning_ros warning_sensor_msgs warning_tf warning_kdl_parser warning_kdl
warning_ros:
	$(SILENT)echo -e "$(INDENT_PRINT) --> $(TRED)Disabling robot state publisher plugin$(TNORMAL) (ROS not available)"
warning_tf:
	$(SILENT)echo -e "$(INDENT_PRINT)--> $(TRED)Disabling robot state publisher plugin$(TNORMAL) (tf not available)"
warning_sensor_msgs:
	$(SILENT)echo -e "$(INDENT_PRINT) --> $(TRED)Disabling robot state publisher plugin$(TNORMAL) (ROS package sensor_msgs not installed)"
warning_kdl_parser:
	$(SILENT)echo -e "$(INDENT_PRINT) --> $(TRED)Disabling robot state publisher plugin$(TNORMAL) (ROS package kdl_parser not installed)"
warning_kdl:
	$(SILENT)echo -e "$(INDENT_PRINT) --> $(TRED)Disabling robot state publisher plugin$(TNORMAL) (ROS package kdl not installed)"
endif

include $(BUILDSYSDIR)/base.mk
