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
#
#           $Id$
# last modified: $Date$
#            by: $Author$
#

BASEDIR = ../../..
include $(BASEDIR)/src/firevision/fvconf.mk

CFLAGS   = $(VISION_CFLAGS)
LDFLAGS  = $(VISION_LDFLAGS)
INCDIRS  = $(VISION_INCDIRS)
LIBDIRS  = $(VISION_LIBDIRS)
LIBS     = $(VISION_LIBS)

LIBS_libfvcams = $(VISION_CAM_LIBS)
OBJS_libfvcams = camera.o			\
		 cameracontrol.o

ifeq ($(HAVE_VISCA_CTRL),1)
OBJS_libfvcams += visca.o
endif
ifeq ($(HAVE_EVID100P_CTRL),1)
OBJS_libfvcams += soony_evid100p_control.o
endif
ifeq ($(HAVE_DPPTU_CTRL),1)
OBJS_libfvcams += dp_ptu.o
endif
ifeq ($(HAVE_FIREWIRE_CAM),1)
OBJS_libfvcams += firewire.o
endif
ifeq ($(HAVE_LEUTRON_CAM),1)
OBJS_libfvcams += leutron.o
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


OBJS_all = $(OBJS_libfvcams)
LIBS_all = $(LIBDIR)/libfvcams.so

include $(BASEDIR)/etc/buildsys/base.mk

