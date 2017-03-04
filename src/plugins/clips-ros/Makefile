#*****************************************************************************
#           Makefile Build System for Fawkes: CLIPS ROS Integration
#                            -------------------
#   Created on Tue Oct 22 18:06:01 2013
#   Copyright (C) 2006-2013 by Tim Niemueller, AllemaniACs RoboCup Team
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
include $(BUILDSYSDIR)/clips.mk

REQ_BOOST_LIBS = system
HAVE_BOOST_LIBS = $(call boost-have-libs,$(REQ_BOOST_LIBS))

LIBS_clips_ros = fawkescore fawkesutils fawkesaspects fawkesclipsaspect \
		 fawkesrosaspect
OBJS_clips_ros = clips_ros_plugin.o clips_ros_thread.o

OBJS_all = $(OBJS_clips_ros)

# Extra files for system-wide install, i.e., "make install"
FILES_clips_files   = $(subst $(SRCDIR)/,,$(wildcard $(SRCDIR)/clips/*.clp))
DESTDIR_clips_files = $(EXEC_SHAREDIR)/clips/clips-ros

ifeq ($(HAVE_CPP11)$(HAVE_ROS)$(HAVE_CLIPS)$(HAVE_BOOST_LIBS)$(call ros-have-pkg,xmlrpcpp),11111)
  CFLAGS  += $(CFLAGS_ROS)  $(CFLAGS_CLIPS) $(call ros-pkg-cflags,xmlrpcpp) $(CFLAGS_CPP11) \
             $(call boost-libs-cflags,$(REQ_BOOST_LIBS))
  LDFLAGS += $(LDFLAGS_ROS) $(LDFLAGS_CLIPS) $(call ros-pkg-lflags,xmlrpcpp) \
             $(call boost-libs-ldflags,$(REQ_BOOST_LIBS))

  ifeq ($(call ros-pkg-version-atleast,xmlrpcpp,1.12.7),1)
    CFLAGS += -DHAVE_NEW_ROS_XMLRPCPP_PATH
  endif

  PLUGINS_all = $(PLUGINDIR)/clips-ros.$(SOEXT)

	INSTALL_extra = clips_files
else
  ifneq ($(HAVE_CPP11),1)
    WARN_TARGETS += warning_cpp11
  endif
  ifneq ($(HAVE_ROS),1)
    WARN_TARGETS += warning_ros
  endif
  ifneq ($(HAVE_CLIPS),1)
    WARN_TARGETS += warning_clips
  endif
  ifneq ($(call ros-have-pkg,xmlrpcpp),1)
    WARN_TARGETS += warning_xmlrpcpp
  endif
  ifneq ($(HAVE_BOOST_LIBS),1)
    WARN_TARGETS_BOOST = $(foreach l,$(REQ_BOOST_LIBS),$(if $(call boost-have-lib,$l),, warning_boost_$l))
  endif
endif

ifeq ($(OBJSSUBMAKE),1)
all: $(WARN_TARGETS) $(WARN_TARGETS_BOOST)
.PHONY: warning_ros warning_clips warning_xmlrpcpp warning_cpp11 $(WARN_TARGETS_BOOST)
warning_ros:
	$(SILENT)echo -e "$(INDENT_PRINT)--> $(TRED)Cannot build clips-ros plugin$(TNORMAL) (ROS not found)"

warning_xmlrpcpp:
	$(SILENT)echo -e "$(INDENT_PRINT)--> $(TRED)Cannot build clips-ros plugin$(TNORMAL) (xmlrpcpp ROS package not found)"

warning_cpp11:
	$(SILENT)echo -e "$(INDENT_PRINT)--> $(TRED)Cannot build clips-ros plugin$(TNORMAL) (C++11 not supported)"

warning_clips:
	$(SILENT)echo -e "$(INDENT_PRINT)--> $(TRED)Cannot build clips-ros plugin$(TNORMAL) ($(CLIPS_ERROR))"

$(WARN_TARGETS_BOOST): warning_boost_%:
	$(SILENT)echo -e "$(INDENT_PRINT)--> $(TRED)Cannot build clips-ros plugin $(TNORMAL) (Boost library $* not found)"
endif

include $(BUILDSYSDIR)/base.mk
