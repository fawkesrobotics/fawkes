#*****************************************************************************
#     Makefile Build System for Fawkes: BlackBoard Synchronization Plugin
#                            -------------------
#   Created on Wed May 16 16:00:45 2012
#   copyright (C) 2006-2012 by Tim Niemueller, AllemaniACs RoboCup Team
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
include $(BUILDCONFDIR)/tf/tf.mk

LIBS_libfawkes_amcl_pf = m
OBJS_libfawkes_amcl_pf = pf/pf.o pf/pf_kdtree.o pf/pf_pdf.o pf/pf_vector.o \
			 pf/eig3.o pf/pf_draw.o

LIBS_libfawkes_amcl_map = m
OBJS_libfawkes_amcl_map = map/map.o map/map_cspace.o map/map_range.o \
			  map/map_store.o map/map_draw.o

LIBS_libfawkes_amcl_sensors = m fawkescore fawkes_amcl_pf fawkes_amcl_map
OBJS_libfawkes_amcl_sensors = sensors/amcl_sensor.o sensors/amcl_odom.o \
			      sensors/amcl_laser.o

LIBS_libfawkes_amcl_utils = fawkescore fawkesconfig fvutils fawkes_amcl_map
OBJS_libfawkes_amcl_utils = amcl_utils.o

LIBS_amcl = fawkescore fawkesutils fawkesaspects fawkesinterface \
	    fawkesblackboard fawkesbaseapp \
	    Laser360Interface Position3DInterface LocalizationInterface \
	    fawkes_amcl_pf fawkes_amcl_map fawkes_amcl_sensors \
	    fawkes_amcl_utils
OBJS_amcl = amcl_plugin.o amcl_thread.o

LIBS_map_lasergen = fawkescore fawkesutils fawkesaspects fawkesinterface \
		    fawkesblackboard  fawkesbaseapp \
		    Laser360Interface Position3DInterface \
		    fawkes_amcl_map fawkes_amcl_utils
OBJS_map_lasergen = map_lasergen_plugin.o map_lasergen_thread.o

OBJS_all   = $(OBJS_amcl) $(OBJS_map_lasergen) $(OBJS_libfawkes_amcl_pf) \
             $(OBJS_libfawkes_amcl_map) $(OBJS_libfawkes_amcl_sensors) \
             $(OBJS_libfawkes_amcl_utils)
LIBS_all    = $(LIBDIR)/libfawkes_amcl_pf.$(SOEXT) \
              $(LIBDIR)/libfawkes_amcl_map.$(SOEXT) \
              $(LIBDIR)/libfawkes_amcl_sensors.$(SOEXT) \
              $(LIBDIR)/libfawkes_amcl_utils.$(SOEXT)
PLUGINS_all = $(PLUGINDIR)/amcl.$(SOEXT) \
              $(PLUGINDIR)/map-lasergen.$(SOEXT) 

# throw exceptions instead of aborting
CFLAGS += -DUSE_ASSERT_EXCEPTION -DUSE_MAP_PUB

ifeq ($(HAVE_TF),1)
  CFLAGS_amcl_thread  = $(CFLAGS) $(CFLAGS_TF)
  CFLAGS_amcl_plugin  = $(CFLAGS_amcl_thread)
  LDFLAGS_amcl       += $(LDFLAGS_TF)
  LIBS_amcl          += fawkestf

  CFLAGS_map_lasergen_thread = $(CFLAGS) $(CFLAGS_TF) $(CFLAGS_CPP11)
  CFLAGS_map_lasergen_plugin = $(CFLAGS_map_lasergen_thread)
  LDFLAGS_map_lasergen      += $(LDFLAGS_TF)
  LIBS_map_lasergen         += fawkestf

  ifeq ($(HAVE_ROS2),1)
    ifeq ($(call ros2-have-pkg,geometry_msgs),1)
      ifeq ($(call ros2-have-pkg,nav_msgs),1)
        CFLAGS_amcl_thread += $(CFLAGS_ROS2) \
          $(call ros2-pkg-cflags,geometry_msgs) \
          $(call ros2-pkg-cflags,nav_msgs)
        CFLAGS_amcl_plugin += $(CFLAGS_ros_thread)
        LDFLAGS_amcl += $(LDFLAGS_ROS2) \
          $(call ros2-pkg-lflags,geometry_msgs) \
          $(call ros2-pkg-lflags,nav_msgs)
        LIBS_amcl += fawkesros2aspect
        OBJS_amcl += ros_thread.o
        CFLAGS_ros_thread = $(CFLAGS_amcl_thread) $(CFLAGS_ROS2)
      else
        WARN_TARGET += warning_nav_msgs
      endif
    else
      WARN_TARGET += warning_geometry_msgs
    endif
  else
    WARN_TARGET += warning_ros
  endif

  PLUGINS_build = $(PLUGINDIR)/amcl.$(SOEXT)
  LIBS_build    = $(LIBS_all)

  ifeq ($(HAVE_CPP11),1)
    PLUGINS_build += $(PLUGINDIR)/map-lasergen.$(SOEXT) 
  else
    WARN_TARGETS += warning_cpp11
  endif
else
  WARN_TARGETS += warning_tf
endif


# Ordering constraints
$(LIBDIR)/libfawkes_amcl_sensors.$(SOEXT): | $(LIBDIR)/libfawkes_amcl_pf.$(SOEXT) $(LIBDIR)/libfawkes_amcl_map.$(SOEXT)
$(PLUGINS_build) $(LIBDIR)/libfawkes_amcl_utils.$(SOEXT): | $(LIBDIR)/libfawkes_amcl_sensors.$(SOEXT) $(LIBDIR)/libfawkes_amcl_pf.$(SOEXT) $(LIBDIR)/libfawkes_amcl_map.$(SOEXT)
$(PLUGINS_build): | $(LIBDIR)/libfawkes_amcl_utils.$(SOEXT)

ifeq ($(OBJSSUBMAKE),1)
all: $(WARN_TARGETS)

.PHONY: warning_tf warning_ros warning_geometry_msgs
warning_tf:
	$(SILENT)echo -e "$(INDENT_PRINT)--> $(TRED)Omitting amcl plugin and libs$(TNORMAL) (tf not available)"
warning_ros:
	$(SILENT)echo -e "$(INDENT_PRINT)--> $(TRED)Omitting ROS interaction$(TNORMAL) (ROS not found)"
warning_geometry_msgs:
	$(SILENT)echo -e "$(INDENT_PRINT)--> $(TRED)Omitting ROS interaction$(TNORMAL) (package geometry_msgs not found)"
warning_nav_msgs:
	$(SILENT)echo -e "$(INDENT_PRINT)--> $(TRED)Omitting ROS interaction$(TNORMAL) (package nav_msgs not found)"
warning_cpp11:
	$(SILENT)echo -e "$(INDENT_PRINT)--> $(TRED)Omitting map-lasergen plugin$(TNORMAL) (C++11 not supported by compiler)"
endif

include $(BUILDSYSDIR)/base.mk
