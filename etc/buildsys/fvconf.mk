#*****************************************************************************
#            Makefile Build System for Fawkes: FireVision Config
#                            -------------------
#   Created on Sun Jan 14 23:00:47 2007
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

ifndef __fvconf_mk_
__fvconf_mk_ := 1

ifndef __buildsys_config_mk_
$(error config.mk must be included before fvconf.mk)
endif

CAMS=LEUTRON FIREWIRE FILELOADER NETWORK SHMEM V4L V4L1 V4L2 BUMBLEBEE2 NAO \
     SWISSRANGER PIKE
CTRLS=EVID100P DPPTU

FVCONFDIR           = $(EXEC_CONFDIR)/firevision
VISION_INCDIRS      =
VISION_CFLAGS       = -D__STDC_LIMIT_MACROS -DFVCONFDIR=\"$(FVCONFDIR)\"

# PTGrey Triclops SDK used for Bumblebee2 stereo processing
TRICLOPS_SDK=$(SYSROOT)/opt/Triclops3.2.0.8-FC3

ifneq ($(wildcard $(SYSROOT)/usr/include/lvsds),)
  HAVE_LEUTRON_CAM    = 1
  HAVE_VISCA_CTRL     = 1
  VISION_LIBDIRs     += $(SYSROOT)/usr/lib/lvsds
  VISION_INCDIRS     += $(SYSROOT)/usr/include/lvsds
  VISION_CAM_LIBS    += lvsds.34
endif
ifeq ($(HAVE_VISCA_CTRL),1)
  HAVE_EVID100P_CTRL  = 1
endif

# check for JPEG lib
ifneq ($(wildcard $(SYSROOT)/usr/include/jpeglib.h $(SYSROOT)/usr/local/include/jpeglib.h $(SYSROOT)/opt/local/include/jpeglib.h),)
  HAVE_LIBJPEG   = 1
  VISION_CFLAGS += -DHAVE_LIBJPEG
endif

ifneq ($(PKGCONFIG),)
  HAVE_LIBDC1394 = $(if $(shell $(PKGCONFIG) --exists 'libdc1394-2'; echo $${?/1/}),1,0)
  HAVE_SDL = $(if $(shell $(PKGCONFIG) --exists 'sdl'; echo $${?/1/}),1,0)
  HAVE_LIBPNG = $(if $(shell $(PKGCONFIG) --exists 'libpng'; echo $${?/1/}),1,0)
  HAVE_LIBV4L2 = $(if $(shell $(PKGCONFIG) --exists 'libv4l2'; echo $${?/1/}),1,0)
  HAVE_LIBUSB = $(if $(shell $(PKGCONFIG) --exists 'libusb'; echo $${?/1/}),1,0)
  HAVE_OPENCV = $(if $(shell $(PKGCONFIG) --exists 'opencv'; echo $${?/1/}),1,0)
  ifneq ($(HAVE_OPENCV),1)
    # Give it another shot, name might contain version
    _OPENCV_ALTERNATE_NAME=$(shell $(PKGCONFIG) --list-all | grep 'opencv' | awk '{ print $$1 }')
    ifneq ($(_OPENCV_ALTERNATE_NAME),)
      OPENCV_VERSION_SUFFIX=$(patsubst opencv%,%,$(_OPENCV_ALTERNATE_NAME))
      HAVE_OPENCV=1
    endif
  endif
endif
ifeq ($(HAVE_LIBDC1394),1)
  HAVE_FIREWIRE_CAM   = 1
  HAVE_BUMBLEBEE2_CAM = 1
  HAVE_PIKE_CAM       = 1
  VISION_CAM_LDFLAGS += $(shell $(PKGCONFIG) --libs 'libdc1394-2')
endif

ifeq ($(HAVE_SDL),1)
  CFLAGS_SDL  = $(shell $(PKGCONFIG) --cflags 'sdl')
  LDFLAGS_SDL = $(shell $(PKGCONFIG) --libs 'sdl')
endif

# check for PNG lib
ifeq ($(HAVE_LIBPNG),1)
  CFLAGS_LIBPNG  = -DHAVE_LIBPNG $(shell $(PKGCONFIG) --cflags 'libpng')
  LDFLAGS_LIBPNG = $(shell $(PKGCONFIG) --libs 'libpng')
endif

ifeq ($(HAVE_LIBPNG),1)
  CFLAGS_LIBV4L2  = -DHAVE_LIBV4L2 $(shell $(PKGCONFIG) --cflags 'libv4l2')
  LDFLAGS_LIBV4L2 = $(shell $(PKGCONFIG) --libs 'libv4l2')
endif

ifeq ($(HAVE_OPENCV),1)
  # Specify LDFLAGS manually, too many libs depending on old Gtk atm
  # (filter-out -lhighgui -lcvaux,(shell $(PKGCONFIG) --libs 'opencv'))
  VERSION_OPENCV          = $(shell $(PKGCONFIG) --modversion 'opencv$(OPENCV_VERSION_SUFFIX)')
  VERSION_SPLITTED_OPENCV = $(call split,.,$(VERSION_OPENCV))
  VERSION_MAJOR_OPENCV    = $(word 1,$(VERSION_SPLITTED_OPENCV))
  VERSION_MINOR_OPENCV    = $(word 2,$(VERSION_SPLITTED_OPENCV))
  CFLAGS_OPENCV      = -DHAVE_OPENCV $(shell $(PKGCONFIG) --cflags 'opencv$(OPENCV_VERSION_SUFFIX)')
  ifeq ($(filter-out 2.0 2.1,$(VERSION_MAJOR_OPENCV).$(VERSION_MINOR_OPENCV)),)
    LDFLAGS_OPENCV     = -lcxcore -lcv
    LDFLAGS_OPENCV_GUI = -lhighgui
  else
    LDFLAGS_OPENCV     = -lopencv_core -lopencv_imgproc -lopencv_ml
    LDFLAGS_OPENCV_GUI = -lopencv_highgui
  endif
endif

ifeq ($(HAVE_LIBUSB),1)
  CFLAGS_LIBUSB  = -DHAVE_LIBUSB $(shell $(PKGCONFIG) --cflags 'libusb')
  LDFLAGS_LIBUSB = $(shell $(PKGCONFIG) --libs 'libusb')
endif

# Check for SwissRanger library
ifneq ($(wildcard $(realpath $(SYSROOT)/usr/include/libMesaSR.h)),)
  ifeq ($(HAVE_LIBUSB),1)
    HAVE_SWISSRANGER_CAM   = 1
    VISION_CAM_LIBS       += mesasr
    CFLAGS_SWISSRANGER     = $(CFLAGS_LIBUSB)
    LDFLAGS_SWISSRANGER    = $(LDFLAGS_LIBUSB)
  endif
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

HAVE_NETWORK_CAM    = 1
HAVE_FILELOADER_CAM = 1
HAVE_SHMEM_CAM      = 1


HAVE_DPPTU_CTRL     = 0
ifeq ($(OS),Linux)
  HAVE_V4L_CAM        = 1
  ifneq ($(wildcard $(SYSROOT)/usr/include/linux/videodev.h),)
    HAVE_V4L1_CAM       = 1
  else
    HAVE_V4L1_FAIL_REASON = v4l1 not available, kernel >= 2.6.38 removed support
  endif
  ifneq ($(wildcard $(SYSROOT)/usr/include/linux/videodev2.h),)
    HAVE_V4L2_CAM       = 1
    ifeq ($(BUILD_TYPE),naocross)
      HAVE_NAO_CAM      = 1
      VISION_CFLAGS    += $(NAOQI_CFLAGS)
    endif
  else
    HAVE_V4L2_FAIL_REASON = v4l2 not available, kernel too old?
  endif
endif


### Need at least one of V4L 1 or 2 for V4L
ifeq ($(HAVE_V4L_CAM),1)
  ifneq ($(HAVE_V4L1_CAM),1)
    ifneq ($(HAVE_V4L2_CAM),1)
      ERROR_TARGETS += error_V4L_CAM
    endif
  endif
endif

### Need V4L2 for NAO
ifeq ($(HAVE_NAO_CAM),1)
  ifneq ($(HAVE_V4L2_CAM),1)
    ERROR_TARGETS += error_NAO_CAM
  endif
endif

### Check for external libraries
ICC_DIR  = $(SYSROOT)$(wildcard /opt/intel/Compiler/*/*)
IPP_DIR  = $(ICC_DIR)/ipp
HAVE_IPP = 0
ifneq ($(wildcard $(realpath $(IPP_DIR))),)
  # Check versions, use first one found
  #IPP_VERSION = $(firstword $(shell ls $(IPP_DIR)))
  # We at least have a IPP, check if it matches our system
  IPP_ARCH = ia32
  ICC_ARCH = ia32
  ifeq ($(ARCH),x86_64)
    IPP_ARCH   = em64t
    ICC_ARCH   = intel64
  endif
  ifneq ($(wildcard $(realpath $(IPP_DIR)/$(IPP_ARCH)/include/ipp.h)),)
    HAVE_IPP = 1
    VISION_CFLAGS  += -DHAVE_IPP
    VISION_LIBS    += pthread m
    ifeq ($(wildcard $(IPP_DIR)/$(IPP_VERSION)/$(IPP_ARCH)/sharedlib/libguide.so),)
      # IPP is used from ICC installation, possibly without actually using icc atm
      VISION_LIBDIRS += $(ICC_DIR)/lib/$(ICC_ARCH)
      VISION_INCDIRS += $(IPP_DIR)/include
    endif
    VISION_LIBDIRS += $(IPP_DIR)/$(IPP_VERSION)/$(IPP_ARCH)/lib
    VISION_LIBDIRS += $(IPP_DIR)/$(IPP_VERSION)/$(IPP_ARCH)/sharedlib
    VISION_INCDIRS += $(IPP_DIR)/$(IPP_VERSION)/$(IPP_ARCH)/include
  endif
endif

## check for SIFT-support (patent-encumbered!)
SIFT_DIR = $(EXTLIBDIR)/sift
ifneq ($(wildcard $(realpath $(SIFT_DIR))),)
  HAVE_SIFT = 1
  LIBS_SIFT = sift
  CFLAGS_SIFT = -DHAVE_SIFT -I$(SIFT_DIR)/include
endif

## check for SURF-support (patent-encumbered!)
SURF_DIR = $(EXTLIBDIR)/surf
ifneq ($(wildcard $(realpath $(SURF_DIR))),)
  ifneq ($(ARCH),x86_64)
    HAVE_SURF = 1
    LIBS_SURF = surf
    CFLAGS_SURF = -DHAVE_SURF -I$(EXTLIBDIR)
  endif
endif

## check for SIFTPP-support (patent-encumbered!)
SIFTPP_DIR = $(EXTLIBDIR)/siftpp
ifneq ($(wildcard $(realpath $(SIFTPP_DIR))),)
  HAVE_SIFTPP = 1
  LIBS_SIFTPP = siftpp
  CFLAGS_SIFTPP = -DHAVE_SIFTPP -I$(EXTLIBDIR)
# -DNDEBUG -DVL_LOWE_STRICT -DVL_USEFASTMATH
endif

# Set to 1 to build shape models
HAVE_SHAPE_MODELS = 1

ifneq ($(wildcard $(TOP_FVSRCDIR)/libs/bulb_calib/bulb*),)
  HAVE_BULB_CREATOR = 1
  VISION_CFLAGS += -DHAVE_BULB_CREATOR
endif

ifneq ($(wildcard $(LIBSRCDIR)/fvutils/rectification),)
  HAVE_RECTINFO = 1
  VISION_CFLAGS += -DHAVE_RECTINFO
endif

VISION_CFLAGS       += $(foreach CAM,$(CAMS),$(if $(subst 0,,$(HAVE_$(CAM)_CAM)),-DHAVE_$(CAM)_CAM))
VISION_CFLAGS       += $(foreach CTRL,$(CTRLS),$(if $(subst 0,,$(HAVE_$(CTRL)_CTRL)),-DHAVE_$(CTRL)_CTRL))

ifeq ($(MAKECMDGOALS),printconf)
VISION_CAM_PRINT     = $(foreach CAM,$(CAMS),$(CAM): $(if $(subst 0,,$(HAVE_$(CAM)_CAM)),"yes","no")\n)
VISION_CTRL_PRINT    = $(foreach CTRL,$(CTRLS),$(CTRL): $(if $(subst 0,,$(HAVE_$(CTRL)_CTRL)),"yes","no")\n)
VISION_LIBS_PRINT    = $(foreach DLIB,LIBJPEG LIBPNG LIBDC1394 SDL TRICLOPS_SDK IPP OPENCV SHAPE_MODELS SIFT SURF SIFTPP,$(DLIB): $(if $(subst 0,,$(HAVE_$(DLIB))),"yes","no")\n)
printconf:
	$(SILENT)echo "Cameras:"
	$(SILENT)echo -e " $(VISION_CAM_PRINT)"
	$(SILENT)echo "Controls:"
	$(SILENT)echo -e " $(VISION_CTRL_PRINT)"
	$(SILENT)echo "Libs:"
	$(SILENT)echo -e " $(VISION_LIBS_PRINT)"
	$(SILENT)echo VISION_LIBDIRS:     $(VISION_LIBDIRS)
	$(SILENT)echo VISION_INCDIRS:     $(VISION_INCDIRS)
	$(SILENT)echo VISION_CAM_LIBS:    $(VISION_CAM_LIBS)
	$(SILENT)echo VISION_CAM_LDFLAGS: $(VISION_CAM_LDFLAGS)
	$(SILENT)echo VISION_CFLAGS:      $(VISION_CFLAGS)
endif

ifneq ($(SRCDIR),)
all: $(ERROR_TARGETS)
.PHONY: error_V4L_CAM error_NAO_CAM
error_V4L_CAM:
	$(SILENT)echo -e "$(INDENT_PRINT)--- $(TRED)If you enable V4L_CAM, you have to enable at least one of V4L1_CAM and V4L2_CAM$(TNORMAL)"
	$(SILENT)exit 1

error_NAO_CAM:
	$(SILENT)echo -e "$(INDENT_PRINT)--- $(TRED)If you enable NAO_CAM, you have to enable V4L2_CAM$(TNORMAL)"
	$(SILENT)exit 1
endif

endif # __fvconf_mk_

