#*****************************************************************************
#            Makefile Build System for Fawkes : FireVision Cams
#                            -------------------
#   Created on Sun Jan 14 18:25:34 2007
#   copyright (C) 2006-2007 by Tim Niemueller, AllemaniACs RoboCup Team
#
#   $Id$
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
include $(BASEDIR)/src/firevision/fvconf.mk

CFLAGS   += $(VISION_CFLAGS)
LDFLAGS  += $(VISION_LDFLAGS)
INCDIRS  += $(VISION_INCDIRS)
LIBDIRS  += $(VISION_LIBDIRS)
LIBS     += $(VISION_LIBS)

LIBS_libfvcams = $(VISION_CAM_LIBS) core utils fvutils
OBJS_libfvcams = camera.o			\
		 cameracontrol.o		\
		 cam_exceptions.o		\
		 factory.o			\
		 controlfactory.o

ifeq ($(HAVE_VISCA_CTRL),1)
OBJS_libfvcams += visca.o
endif
ifeq ($(HAVE_EVID100P_CTRL),1)
OBJS_libfvcams += sony_evid100p_control.o
endif
ifeq ($(HAVE_DPPTU_CTRL),1)
OBJS_libfvcams += dp_ptu.o
endif
ifeq ($(HAVE_FIREWIRE_CAM),1)
OBJS_libfvcams += firewire.o
endif
ifeq ($(HAVE_BUMBLEBEE2_CAM),1)
OBJS_libfvcams += bumblebee2.o
endif
ifeq ($(HAVE_LEUTRON_CAM),1)
OBJS_libfvcams += leutron.o
# Leutron headers are in a bad shape...
CFLAGS += -Wno-packed -Wno-attributes -Wno-non-virtual-dtor
endif
ifeq ($(HAVE_V4L_CAM),1)
OBJS_libfvcams += v4l.o
endif
ifeq ($(HAVE_FILELOADER_CAM),1)
OBJS_libfvcams += fileloader.o
endif
ifeq ($(HAVE_NETWORK_CAM),1)
OBJS_libfvcams += net.o
endif
ifeq ($(HAVE_SHMEM_CAM),1)
OBJS_libfvcams += shmem.o
endif

ifneq ($(HAVE_FIREWIRE_CAM),1)
  ifneq ($(realpath $(SRCDIR)/firewire.cpp),)
    WARN_TARGETS += warning_firewire
  endif
endif
ifneq ($(HAVE_LEUTRON_CAM),1)
  ifeq ($(BUILD_TYPE),rcsoftx)
    WARN_TARGETS += warning_leutron
  endif
endif
ifneq ($(HAVE_LIBJPEG),1)
  WARN_TARGETS += warning_libjpeg
endif

OBJS_all = $(OBJS_libfvcams)
LIBS_all = $(LIBDIR)/libfvcams.so

# Use . instead of nothing here in this pattern because we already included
# the config in fvconf.mk and thus SRCDIR is set!
ifeq ($(OBJSSUBMAKE),1)
all: $(WARN_TARGETS)

.PHONY: warning_firewire
warning_firewire:
	$(SILENT)echo -e "$(INDENT_PRINT)--- $(TRED)No Firewire support$(TNORMAL) (install libdc1394-devel)";

.PHONY: warning_leutron
warning_leutron:
	$(SILENT)echo -e "$(INDENT_PRINT)--- $(TRED)No Leutron support$(TNORMAL) (install Leutron SDK)";

.PHONY: warning_libjpeg
warning_libjpeg:
	$(SILENT)echo -e "$(INDENT_PRINT)--- $(TRED)No JPEG support in file loader$(TNORMAL) (install libjpeg[-devel])";
endif

include $(BASEDIR)/etc/buildsys/base.mk

