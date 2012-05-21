#*****************************************************************************
#            Makefile Build System for Fawkes : FireVision Filters
#                            -------------------
#   Created on Wed Mar 14 19:02:12 2007
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

ifeq ($(HAVE_IPP)$(HAVE_OPENCV),00)
  # We neither IPP nor OpenCV, hence we have to eliminate some filters
  IPPI_FILTERS  = $(wildcard $(SRCDIR)/morphology/*.cpp)
  ALLFILES=$(realpath $(wildcard $(SRCDIR)/*.cpp $(SRCDIR)/*/*.cpp $(SRCDIR)/*/*/*.cpp))
  ifneq ($(ALLFILES),)
    IPPI_FILTERS += $(shell grep -rl ippi.h $(ALLFILES))
  endif
else
  ifeq ($(HAVE_IPP),1)
    LIBS_libfvfilters += ippi ippcore guide pthread
  else
    CFLAGS += $(CFLAGS_OPENCV)
    LDFLAGS += $(LDFLAGS_OPENCV)
  endif
endif

OBJS_libfvfilters := $(patsubst %.cpp,%.o,$(filter-out $(IPPI_FILTERS:$(SRCDIR)/%=%),$(subst $(SRCDIR)/,,$(realpath $(filter-out $(wildcard $(SRCDIR)/qa/*.cpp),$(wildcard $(SRCDIR)/*.cpp $(SRCDIR)/*/*.cpp $(SRCDIR)/*/*/*.cpp))))))
LIBS_libfvfilters += m fawkescore fawkesutils fvutils
HDRS_libfvfilters = nothing.h $(patsubst %.o,%.h,$(OBJS_libfvfilters))

OBJS_all = $(OBJS_libfvfilters)
LIBS_all = $(LIBDIR)/libfvfilters.so

ifeq ($(OBJSSUBMAKE),1)
.PHONY: print_unsupported
print_unsupported:
	$(SILENT)if [ "$(HAVE_IPP)" != "1" ]; then \
		for f in $(IPPI_FILTERS:$(SRCDIR)/%.cpp=%); do \
			echo -e "$(INDENT_PRINT)--- $(TRED)Omitting $$f filter$(TNORMAL) (Neither IPP nor OpenCV found)"; \
		done; \
	fi \

all: print_unsupported
endif


include $(BUILDSYSDIR)/base.mk

