#*****************************************************************************
#        Makefile Build System for Fawkes : FireVision Classifiers
#                            -------------------
#   Created on Mon Mar 19 15:12:35 2007
#   Copyright (C) 2006-2007 by Tim Niemueller, AllemaniACs RoboCup Team
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
include $(BUILDSYSDIR)/fvconf.mk

CFLAGS   += $(VISION_CFLAGS)
LDFLAGS  += $(VISION_LDFLAGS)
INCDIRS  += $(VISION_INCDIRS)
LIBDIRS  += $(VISION_LIBDIRS)
LIBS     += $(VISION_LIBS)

ifeq ($(HAVE_OPENCV),1)
  CFLAGS  += $(CFLAGS_OPENCV)
  LDFLAGS += $(LDFLAGS_OPENCV)
  ifeq ($(filter-out 2.0 2.1,$(VERSION_MAJOR_OPENCV).$(VERSION_MINOR_OPENCV)),)
    LDFLAGS += -lcv
  else
    LDFLAGS += -lopencv_objdetect
  endif
else
  WARN_TARGETS += warning_opencv
  FILTER_OUT += faces.cpp
endif

ifeq ($(HAVE_SIFT),1)
  CFLAGS += $(CFLAGS_SIFT)
  CLASSIFIER_EXTRA_LIBS += $(LIBS_SIFT)
else
  # SIFT is considered truely optional, no warning
  #WARN_TARGETS += warning_sift
  FILTER_OUT += sift.cpp
endif

ifeq ($(HAVE_SURF),1)
  CFLAGS += $(CFLAGS_SURF)
  CLASSIFIER_EXTRA_LIBS += $(LIBS_SURF)
else
  # SURF is considered truely optional, no warning
  #WARN_TARGETS += warning_surf
  FILTER_OUT += surf.cpp
endif

ifeq ($(HAVE_SIFTPP),1)
  CFLAGS += $(CFLAGS_SIFTPP)
  CLASSIFIER_EXTRA_LIBS += $(LIBS_SIFTPP)
else
  # SIFTPP is considered truely optional, no warning
  #WARN_TARGETS += warning_siftpp
  FILTER_OUT += siftpp.cpp
endif

ifeq ($(OBJSSUBMAKE),1)
all: $(WARN_TARGETS)
warning_opencv:
	$(SILENT)echo -e "$(INDENT_PRINT)--> $(TRED)Omitting face detector$(TNORMAL) (OpenCV not installed)"
warning_sift:
	$(SILENT)echo -e "$(INDENT_PRINT)--> $(TRED)Omitting SIFT feature detector$(TNORMAL) (SIFT not present)"
warning_surf:
	$(SILENT)echo -e "$(INDENT_PRINT)--> $(TRED)Omitting SURF-feature detector$(TNORMAL) (SURF not present)"
warning_siftpp:
	$(SILENT)echo -e "$(INDENT_PRINT)--> $(TRED)Omitting SIFTPP-feature detector$(TNORMAL) (SIFTPP not present)"
endif

OBJS_libfvclassifiers := $(patsubst %.cpp,%.o,$(filter-out $(FILTER_OUT),$(patsubst qa/%,,$(subst $(SRCDIR)/,,$(realpath $(wildcard $(SRCDIR)/*.cpp $(SRCDIR)/*/*.cpp $(SRCDIR)/*/*/*.cpp))))))
LIBS_libfvclassifiers := fawkescore fvutils $(CLASSIFIER_EXTRA_LIBS)
HDRS_libfvclassifiers = $(patsubst %.o,%.h,$(OBJS_libfvclassifiers))

OBJS_all = $(OBJS_libfvclassifiers)
LIBS_all = $(LIBDIR)/libfvclassifiers.so

include $(BUILDSYSDIR)/base.mk

