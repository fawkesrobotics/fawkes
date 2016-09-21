#*****************************************************************************
#           Makefile Build System for Fawkes: MongoDB Logging Plugin
#                            -------------------
#   Created on Fri Dec 14 00:43:40 2012
#   Copyright (C) 2006-2012 by Tim Niemueller, AllemaniACs RoboCup Team
#
#*****************************************************************************
#
#   This program is free software; you can redistribute it and/or modify
#   it under the terms of the GNU General Public License as published by
#   the Free Software Foundation; either version 2 of the License, or
#   (at your option) any later version.
#
#*****************************************************************************

BASEDIR = ../../../..

include $(BASEDIR)/etc/buildsys/config.mk
include $(BUILDSYSDIR)/pcl.mk
include $(BUILDCONFDIR)/tf/tf.mk
include $(BASEDIR)/src/plugins/mongodb/mongodb.mk

LIBS_ffmongodb_save_imgs = fawkescore fawkesaspects fawkesblackboard fawkesinterface \
			   fawkesutils fawkeslogging fawkesmongodbaspect fvutils \
			   fawkestf
OBJS_ffmongodb_save_imgs = ffmongodb_save_imgs.o

ifeq ($(DISTRO),debian)
	LIBS_ffmongodb_save_imgs += crypto ssl
endif

CFLAGS  += $(CFLAGS_MONGODB)
LDFLAGS += $(LDFLAGS_MONGODB)

OBJS_all    = $(OBJS_ffmongodb_save_imgs)

ifeq ($(HAVE_MONGODB),1)
  CFLAGS_ffmongodb_save_imgs  = $(CFLAGS) $(CFLAGS_MONGODB) -Wno-deprecated
  LDFLAGS_ffmongodb_save_imgs = $(LDFLAGS) $(LDFLAGS_MONGODB)
  BINS_all = $(BINDIR)/ffmongodb-save-imgs
  MANPAGES_all =  $(MANDIR)/man1/ffmongodb-save-imgs.1
else
  WARN_TARGETS += warning_mongodb
endif

ifeq ($(OBJSSUBMAKE),1)
all: $(WARN_TARGETS)

.PHONY: warning_mongodb
warning_mongodb:
	$(SILENT)echo -e "$(INDENT_PRINT)--> $(TRED)Omitting MongoDB Logging Tools$(TNORMAL) (mongodb[-devel] not installed)"
endif

include $(BUILDSYSDIR)/base.mk

