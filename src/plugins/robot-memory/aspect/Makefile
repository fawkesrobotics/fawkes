#*****************************************************************************
#           Makefile Build System for Fawkes: Robot-Memory Aspect
#                            -------------------
#   Created on Tue Aug 23 14:32:12 2016
#   Copyright (C) 2016 Frederik Zwilling
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
include $(LIBSRCDIR)/utils/utils.mk
include $(BASEDIR)/src/plugins/mongodb/mongodb.mk

LIBS_libfawkesrobotmemory = fawkescore fawkesaspects fawkesutils
OBJS_libfawkesrobotmemory = $(patsubst %.cpp,%.o,$(patsubst qa/%,,$(subst $(SRCDIR)/,,$(realpath $(wildcard $(SRCDIR)/*.cpp)))))

OBJS_all = $(OBJS_libfawkesrobotmemory)
LIBS_all = $(LIBDIR)/libfawkesrobotmemory.so

ifeq ($(HAVE_MONGODB)$(HAVE_CPP17),11)
  CFLAGS += $(CFLAGS_MONGODB) $(CFLAGS_CPP17)
  LDFLAGS += $(LDFLAGS_MONGODB)

  LIBS_build = $(LIBS_all)
else
  ifneq ($(HAVE_MONGODB),1)
    WARN_TARGETS += warning_mongodb
  endif
  ifneq ($(HAVE_CPP17),1)
    WARN_TARGETS += warning_cpp17
  endif
endif

ifeq ($(OBJSSUBMAKE),1)
all: $(WARN_TARGETS)

.PHONY: warning_mongodb warning_cpp17
warning_mongodb:
	$(SILENT)echo -e "$(INDENT_PRINT)--> $(TRED)Omitting RobotMemoryAspect$(TNORMAL) (mongodb[-devel] not installed)"
warning_cpp17:
	$(SILENT)echo -e "$(INDENT_PRINT)--> $(TRED)Omitting RobotMemoryAspect$(TNORMAL) (C++17 required)"
endif

include $(BUILDSYSDIR)/base.mk

