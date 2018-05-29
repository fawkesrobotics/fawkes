#*****************************************************************************
#                      Makefile Build System for Fawkes
#                            -------------------
#   Created on Mon 10 Jul 2017 18:16:01 CEST
#   Copyright (C) 2017 by Till Hofmann <hofmann@kbsg.rwth-aachen.de>
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
include $(BUILDCONFDIR)/tf/tf.mk

LIBS_laser_calibration = stdc++ m fawkescore fawkesutils fawkesblackboard \
		fawkesconfig fawkesinterface fawkesnetcomm fawkestf fawkespcl_utils \
		Laser360Interface Laser720Interface Laser1080Interface MotorInterface

OBJS_laser_calibration = laser_calibration.o main.o \
                         roll_calibration.o pitch_calibration.o \
                         yaw_calibration.o time_offset_calibration.o
OBJS_all = $(OBJS_laser_calibration)

REQUIRED_PCL_LIBS = common filters registration

ifeq ($(HAVE_TF),1)
  ifeq ($(HAVE_PCL),1)
    ifeq ($(call pcl-have-libs,$(REQUIRED_PCL_LIBS)),1)
      CFLAGS += $(CFLAGS_PCL) $(call pcl-libs-cflags,$(REQUIRED_PCL_LIBS)) \
                $(CFLAGS_TF)
      LDFLAGS += $(LDFLAGS_PCL) $(call pcl-libs-ldflags,$(REQUIRED_PCL_LIBS)) \
                 $(LDFLAGS_TF)
      BINS_all += $(BINDIR)/laser_calibration
    else
      WARN_TARGETS += warning_pcl_libs
    endif
  else
    WARN_TARGETS += warning_pcl
  endif
else
  WARN_TARGETS += warning_tf
endif

ifeq ($(OBJSSUBMAKE),1)
all: $(WARN_TARGETS)
.PHONY: warning_tf warning_pcl warning_pcl_libs
warning_tf:
	$(SILENT)echo -e "$(INDENT_PRINT)--> $(TRED)Omitting laser calibration tool (TF not available)"
warning_pcl:
	$(SILENT)echo -e "$(INDENT_PRINT)--> $(TRED)Omitting laser calibration tool (PCL not available)"
warning_pcl_libs:
	$(SILENT)echo -e "$(INDENT_PRINT)--> $(TRED)Omitting laser calibration tool (not all PCL libs available, required: $(REQUIRED_PCL_LIBS))"
endif

include $(BUILDSYSDIR)/base.mk
