#*****************************************************************************
#            Makefile Build System for Fawkes: FireVision Config
#                            -------------------
#   Created on Sun Jan 14 23:00:47 2007
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
#
#           $Id$
# last modified: $Date$
#            by: $Author$
#
#*****************************************************************************

ifndef __fvconf_mk_
__fvconf_mk_ := 1

include $(BASEDIR)/etc/buildsys/config.mk

CAMS=LEUTRON FIREWIRE FILELOADER NETWORK SHMEM V4L BUMBLEBEE2
CTRLS=EVID100P DPPTU

FVBASEDIR           = $(BASEDIR)/src/firevision
FVCONFDIR           = $(CONFDIR)/firevision
VISION_INCDIRS      = $(realpath $(FVBASEDIR))
VISION_CFLAGS       = -D__STDC_LIMIT_MACROS -DFVCONFDIR=\"$(FVCONFDIR)\"

# PTGrey Triclops SDK used for Bumblebee2 stereo processing
TRICLOPS_SDK=/opt/Triclops3.2.0.8-FC3

ifneq ($(wildcard /usr/include/lvsds),)
HAVE_LEUTRON_CAM    = 1
HAVE_VISCA_CTRL     = 1
VISION_LIBDIRs     += /usr/lib/lvsds
VISION_INCDIRS     += /usr/include/lvsds
VISION_CAM_LIBS    += lvsds.34
endif
ifeq ($(HAVE_VISCA_CTRL),1)
HAVE_EVID100P_CTRL  = 1
endif

# check for JPEG lib
ifneq ($(wildcard /usr/include/jpeglib.h /usr/local/include/jpeglib.h),)
  HAVE_LIBJPEG   = 1
  VISION_CFLAGS += -DHAVE_LIBJPEG
endif

# check for PNG lib
ifneq ($(wildcard /usr/include/png.h /usr/local/include/png.h),)
  HAVE_LIBPNG    = 1
  VISION_CFLAGS += -DHAVE_LIBPNG
endif

ifneq ($(PKGCONFIG),)
  HAVE_LIBDC1394 = $(if $(shell $(PKGCONFIG) --exists 'libdc1394-2'; echo $${?/1/}),1,0)
  HAVE_SDL = $(if $(shell $(PKGCONFIG) --exists 'sdl'; echo $${?/1/}),1,0)
endif
ifeq ($(HAVE_LIBDC1394),1)
  ifneq ($(wildcard $(realpath $(FVBASEDIR)/cams/firewire.h)),)
    HAVE_FIREWIRE_CAM   = 1
    ifneq ($(wildcard $(realpath $(FVBASEDIR)/cams/bumblebee2.h)),)
      HAVE_BUMBLEBEE2_CAM = 1
    endif
  endif
  VISION_CAM_LIBS    += $(subst -l,,$(shell $(PKGCONFIG) --libs 'libdc1394-2'))
endif

ifeq ($(HAVE_SDL),1)
  CFLAGS_SDL  = $(shell $(PKGCONFIG) --cflags 'sdl')
  LDFLAGS_SDL = $(shell $(PKGCONFIG) --libs 'sdl')
endif

# Check if we have PGR Triclops SDK, build Bumblebee2 if we have it
ifeq ($(ARCH),x86_64)
  TRICLOPS_SDK_ERR="Triclops SDK not available on 64-bit systems"
else
  ifneq ($(HAVE_BUMBLEBEE2_CAM),1)
    TRICLOPS_SDK_ERR="Bumblebee2 camera not available"
  else
    ifneq ($(wildcard $(realpath $(TRICLOPS_SDK)/include/triclops.h)),)
      ifneq ($(wildcard $(realpath $(TRICLOPS_SDK)/lib/libtriclops.so)),)
        HAVE_TRICLOPS_SDK = 1
        TRICLOPS_SDK_INCDIRS += $(TRICLOPS_SDK)/include
        TRICLOPS_SDK_LIBDIRS += $(TRICLOPS_SDK)/lib
        TRICLOPS_SDK_LIBS    += triclops
        VISION_CFLAGS += -DHAVE_TRICLOPS_SDK
      else
        TRICLOPS_SDK_ERR = "shared lib not created, use \"make triclops\" in fvstereo"
      endif
    else
      TRICLOPS_SDK_ERR = "Triclops SDK not installed"
    endif
  endif
endif

ifneq ($(wildcard $(realpath $(FVBASEDIR)/cams/net.h)),)
  HAVE_NETWORK_CAM    = 1
endif
ifneq ($(wildcard $(realpath $(FVBASEDIR)/cams/fileloader.h)),)
  HAVE_FILELOADER_CAM = 1
endif
ifneq ($(wildcard $(realpath $(FVBASEDIR)/cams/shmem.h)),)
  HAVE_SHMEM_CAM      = 1
endif
HAVE_DPPTU_CTRL     = 0
HAVE_V4L_CAM        = 0

### Check for external libraries
IPP_DIR  = /opt/intel/ipp
HAVE_IPP = 0
ifneq ($(wildcard $(realpath $(IPP_DIR))),)
  # Check versions, use first one found
  IPP_VERSION = $(firstword $(shell ls $(IPP_DIR)))
  # We at least have a IPP, check if it matches our system
  INTEL_ARCH = ia32
  ifeq ($(ARCH),x86_64)
    INTEL_ARCH = em64t
    IPP_ARCH   = em64t
  endif
  ifneq ($(wildcard $(realpath $(IPP_DIR)/$(IPP_VERSION)/$(INTEL_ARCH)/include/ipp.h)),)
    HAVE_IPP = 1
    VISION_CFLAGS  += -DHAVE_IPP
    VISION_LIBDIRS += $(IPP_DIR)/$(IPP_VERSION)/$(INTEL_ARCH)/sharedlib
    VISION_INCDIRS += $(IPP_DIR)/$(IPP_VERSION)/$(INTEL_ARCH)/include
  endif
endif

HAVE_OPENCV = $(if $(shell $(PKGCONFIG) --exists 'opencv'; echo $${?/1/}),1,0)
ifeq ($(HAVE_OPENCV),1)
  CFLAGS_OPENCV = -DHAVE_OPENCV $(shell $(PKGCONFIG) --cflags 'opencv')
  LDFLAGS_OPENCV = $(shell $(PKGCONFIG) --libs 'opencv')
endif

# Set to 1 to build shape models
HAVE_SHAPE_MODELS = 1

ifneq ($(wildcard $(realpath $(FVBASEDIR)/fvutils/rectification)),)
  HAVE_RECTINFO = 1
  VISION_CFLAGS += -DHAVE_RECTINFO
endif

VISION_CFLAGS       += $(foreach CAM,$(CAMS),$(if $(subst 0,,$(HAVE_$(CAM)_CAM)),-DHAVE_$(CAM)_CAM))
VISION_CFLAGS       += $(foreach CTRL,$(CTRLS),$(if $(subst 0,,$(HAVE_$(CTRL)_CTRL)),-DHAVE_$(CTRL)_CTRL))

ifeq ($(MAKECMDGOALS),printconf)
VISION_CAM_PRINT     = $(foreach CAM,$(CAMS),$(CAM): $(if $(subst 0,,$(HAVE_$(CAM)_CAM)),"yes","no")\n)
VISION_CTRL_PRINT    = $(foreach CTRL,$(CTRLS),$(CTRL): $(if $(subst 0,,$(HAVE_$(CTRL)_CTRL)),"yes","no")\n)
VISION_LIBS_PRINT    = $(foreach DLIB,LIBJPEG LIBPNG LIBDC1394 SDL TRICLOPS_SDK IPP OPENCV SHAPE_MODELS,$(DLIB): $(if $(subst 0,,$(HAVE_$(DLIB))),"yes","no")\n)
printconf:
	$(SILENT)echo "Cameras:"
	$(SILENT)echo -e " $(VISION_CAM_PRINT)"
	$(SILENT)echo "Controls:"
	$(SILENT)echo -e " $(VISION_CTRL_PRINT)"
	$(SILENT)echo "Libs:"
	$(SILENT)echo -e " $(VISION_LIBS_PRINT)"
	$(SILENT)echo VISION_LIBDIRS:  $(VISION_LIBDIRS)
	$(SILENT)echo VISION_INCDIRS:  $(VISION_INCDIRS)
	$(SILENT)echo VISION_CAM_LIBS: $(VISION_CAM_LIBS)
	$(SILENT)echo VISION_CFLAGS:   $(VISION_CFLAGS)
endif

endif # __fvconf_mk_

