#*****************************************************************************
#            Makefile Build System for Fawkes: IMU Plugin
#                            -------------------
#   Created on Sun Jun 22 19:28:24 2014
#   Copyright (C) 2006-2014 by Tim Niemueller, AllemaniACs RoboCup Team
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

LIBS_imu = m pthread fawkescore fawkesutils fawkesaspects fawkesblackboard \
	     fawkesinterface IMUInterface

CFLAGS  += $(CFLAGS_CPP11)

# Enable for time measurements
#CFLAGS += -DUSE_TIMETRACKER

OBJS_imu = imu_plugin.o acquisition_thread.o sensor_thread.o

OBJS_all = $(OBJS_imu)

ifeq ($(HAVE_CRUIZCORE),1)
  OBJS_imu += imu_cruizcore_xg1010.o
  CFLAGS  += $(CFLAGS_CRUIZCORE)
  LDFLAGS += $(LDFLAGS_CRUIZCORE)
else
  WARN_TARGETS_CRUIZCORE_BOOST = $(foreach l,$(CRUIZCORE_REQ_BOOST_LIBS),$(if $(call boost-have-lib,$l),, warning_cruizcore_boost_$l))
  ifneq ($(HAVE_TF),1)
    WARN_TARGETS += warning_tf
  endif
endif

ifneq ($(HAVE_CRUIZCORE),)
  PLUGINS_all = $(PLUGINDIR)/imu.so
else
  WARN_TARGETS += warning_nolib
endif


ifeq ($(OBJSSUBMAKE),1)
all: $(WARN_TARGETS)

.PHONY: warning_nolib warning_tf $(WARN_TARGETS_BOOST)
warning_nolib:
	$(SILENT)echo -e "$(INDENT_PRINT)--> $(TRED)Omitting IMU Plugin$(TNORMAL) (No hardware access library found)"
warning_tf:
	$(SILENT)echo -e "$(INDENT_PRINT)--> $(TRED)No support for CruizCore$(TNORMAL) (tf not available)"

$(WARN_TARGETS_CRUIZCORE_BOOST): warning_cruizcore_boost_%:
	$(SILENT)echo -e "$(INDENT_PRINT)--> $(TRED)No support for CruizCore$(TNORMAL) (Boost library $* not found)"
endif

include $(BUILDSYSDIR)/base.mk

