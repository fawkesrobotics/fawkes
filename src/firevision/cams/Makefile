#*****************************************************************************
#            Makefile Build System for Fawkes : FireVision Cams
#                            -------------------
#   Created on Sun Jan 14 18:25:34 2007
#   copyright (C) 2006-2007 by Tim Niemueller, AllemaniACs RoboCup Team
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
include $(FVSRCDIR)/fvconf.mk

CFLAGS   += $(VISION_CFLAGS)
LDFLAGS  += $(VISION_LDFLAGS)
INCDIRS  += $(VISION_INCDIRS)
LIBDIRS  += $(VISION_LIBDIRS)
LIBS     += $(VISION_LIBS)

LIBS_libfvcams = $(VISION_CAM_LIBS) fawkescore fawkesutils fvutils
OBJS_libfvcams = camera.o          \
                 control/control.o \
                 control/color.o   \
                 control/image.o   \
                 control/pantilt.o \
                 control/focus.o   \
                 control/zoom.o    \
                 control/effect.o  \
                 control/source.o  \
                 control/dummy.o   \
                 cam_exceptions.o  \
                 factory.o         \
                 control/factory.o

ifeq ($(HAVE_VISCA_CTRL),1)
OBJS_libfvcams += control/visca.o
endif
ifeq ($(HAVE_EVID100P_CTRL),1)
OBJS_libfvcams += control/sony_evid100p.o
endif
ifeq ($(HAVE_DPPTU_CTRL),1)
OBJS_libfvcams += control/dp_ptu.o
endif
ifeq ($(HAVE_FIREWIRE_CAM),1)
OBJS_libfvcams += firewire.o
endif
ifeq ($(HAVE_BUMBLEBEE2_CAM),1)
OBJS_libfvcams += bumblebee2.o
endif
ifeq ($(HAVE_PIKE_CAM),1)
OBJS_libfvcams += pike.o
endif
ifeq ($(HAVE_LEUTRON_CAM),1)
OBJS_libfvcams += leutron.o
# Leutron headers are in a bad shape...
CFLAGS += -Wno-packed -Wno-attributes -Wno-non-virtual-dtor
endif
ifeq ($(HAVE_V4L1_CAM),1)
OBJS_libfvcams += v4l1.o
endif
ifeq ($(HAVE_V4L2_CAM),1)
OBJS_libfvcams += v4l2.o
endif
ifeq ($(HAVE_V4L_CAM),1)
OBJS_libfvcams += v4l.o
endif
ifeq ($(HAVE_NAO_CAM),1)
OBJS_libfvcams += nao.o
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
ifeq ($(HAVE_SWISSRANGER_CAM),1)
OBJS_libfvcams += swissranger.o
endif
ifeq ($(HAVE_LIBV4L2),1)
CFLAGS  += $(CFLAGS_LIBV4L2)
LDFLAGS += $(LDFLAGS_LIBV4L2)
endif

ifneq ($(HAVE_FIREWIRE_CAM),1)
  ifneq ($(realpath $(SRCDIR)/firewire.cpp),)
    WARN_TARGETS += warning_firewire
  endif
endif
ifneq ($(HAVE_V4L2_CAM),1)
  ifneq ($(HAVE_V4L2_FAIL_REASON),)
    WARN_TARGETS += warning_v4l2
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
HDRS_libfvcams = $(patsubst %.o,%.h,$(OBJS_libfvcams))

OBJS_all = $(OBJS_libfvcams)
LIBS_all = $(LIBDIR)/libfvcams.so

# Use . instead of nothing here in this pattern because we already included
# the config in fvconf.mk and thus SRCDIR is set!
ifeq ($(OBJSSUBMAKE),1)
all: $(WARN_TARGETS)

.PHONY: warning_firewire warning_leutron warning_libjpeg warning_v4l2
warning_firewire:
	$(SILENT)echo -e "$(INDENT_PRINT)--- $(TRED)No Firewire support$(TNORMAL) (install libdc1394-devel)";

warning_leutron:
	$(SILENT)echo -e "$(INDENT_PRINT)--- $(TRED)No Leutron support$(TNORMAL) (install Leutron SDK)";

warning_libjpeg:
	$(SILENT)echo -e "$(INDENT_PRINT)--- $(TRED)No JPEG support in file loader$(TNORMAL) (install libjpeg[-devel])";

warning_v4l2:
	$(SILENT)echo -e "$(INDENT_PRINT)--- $(TRED)No V4L2 support$(TNORMAL) ($(HAVE_V4L2_FAIL_REASON))";
endif

include $(BUILDSYSDIR)/base.mk

