#*****************************************************************************
#            Makefile Build System for Fawkes: Laser Plugin
#                            -------------------
#   Created on Wed Oct 08 13:39:39 2008
#   Copyright (C) 2006-2008 by Tim Niemueller, AllemaniACs RoboCup Team
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

LIBS_laser = m fawkescore fawkesutils fawkesaspects fawkesblackboard \
	     fawkesinterface Laser360Interface Laser720Interface \
	     Laser1080Interface

OBJS_laser = laser_plugin.o acquisition_thread.o sensor_thread.o

CFLAGS  += $(CFLAGS_LIBUDEV)
LDFLAGS += $(LDFLAGS_LIBUDEV)

ifeq ($(HAVE_LIBPCAN),1)
  OBJS_laser += lase_edl_aqt.o
  LIBS_laser += $(LIBS_LIBPCAN)
  CFLAGS     += $(CFLAGS_LIBPCAN)
else
  WARN_TARGETS += warning_libpcan
endif

ifeq ($(HAVE_URG),1)
  OBJS_laser += urg_aqt.o
  LIBS_laser += $(LIBS_URG)
  CFLAGS     += $(CFLAGS_URG)
else
  WARN_TARGETS += warning_urg
endif

ifeq ($(HAVE_URG_GBX),1)
  OBJS_laser += urg_gbx_aqt.o
  CFLAGS     += $(CFLAGS_URG_GBX)
  LDFLAGS    += $(LDFLAGS_URG_GBX)
else
  WARN_TARGETS += warning_urg_gbx
endif

# Yes, only a single zero, boost.mk returns empty string on fail
ifneq ($(HAVE_LIBUSB)$(HAVE_SICK_TIM55X_BOOST_LIBS),0)
  OBJS_laser += sick_tim55x_common_aqt.o
endif
ifeq ($(HAVE_LIBUSB),1)
  OBJS_laser += sick_tim55x_usb_aqt.o
  CFLAGS     += $(CFLAGS_LIBUSB)
  LDFLAGS    += $(LDFLAGS_LIBUSB)
else
  WARN_TARGETS += warning_libusb
endif
ifeq ($(HAVE_SICK_TIM55X_BOOST_LIBS),1)
  OBJS_laser += sick_tim55x_ethernet_aqt.o
  CFLAGS     += $(CFLAGS_SICK_TIM55X_BOOST)
  LDFLAGS    += $(LDFLAGS_SICK_TIM55X_BOOST)
else
  WARN_TARGETS_SICK_TIM55X_BOOST = $(foreach l,$(SICK_TIM55X_REQ_BOOST_LIBS),$(if $(call boost-have-lib,$l),, warning_sick_55x_boost_$l))
endif

ifneq ($(HAVE_LIBPCAN)$(HAVE_URG)$(HAVE_URG_GBX)$(HAVE_LIBUSB),0000)
  PLUGINS_all = $(PLUGINDIR)/laser.so
else
  WARN_TARGETS += warning_nolib
endif

OBJS_all = $(OBJS_laser)

ifeq ($(OBJSSUBMAKE),1)
all: $(WARN_TARGETS)

.PHONY: warning_nolib warning_libpcan warning_urg warning_urg_gbx
warning_nolib:
	$(SILENT)echo -e "$(INDENT_PRINT)--> $(TRED)Omitting Laser Plugin$(TNORMAL) (No hardware access library found)"

warning_libpcan:
	$(SILENT)echo -e "$(INDENT_PRINT)--> $(TYELLOW)No support for Lase EDL$(TNORMAL) (libpcan not found)"

warning_urg:
	$(SILENT)echo -e "$(INDENT_PRINT)--> $(TYELLOW)No support for Hokuyo URG$(TNORMAL) (urg library not found)"

warning_urg_gbx:
	$(SILENT)echo -e "$(INDENT_PRINT)--> $(TYELLOW)No support for Hokuyo via Gearbox library$(TNORMAL) (Gearbox library not found)"

warning_libusb:
	$(SILENT)echo -e "$(INDENT_PRINT)--> $(TYELLOW)No support for Sick TiM55x-USB$(TNORMAL) (libusb not found)"

$(WARN_TARGETS_SICK_TIM55X_BOOST): warning_sick_tim55x_boost_%:
	$(SILENT)echo -e "$(INDENT_PRINT)--> $(TYELLOW)No suport for TiM55x-Ethernet$(TNORMAL) (Boost library $* not found)"

endif

include $(BUILDSYSDIR)/base.mk

