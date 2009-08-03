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
include $(BASEDIR)/src/firevision/fvconf.mk

CFLAGS   += $(VISION_CFLAGS)
LDFLAGS  += $(VISION_LDFLAGS)
INCDIRS  += $(VISION_INCDIRS)
LIBDIRS  += $(VISION_LIBDIRS)
LIBS     += $(VISION_LIBS)

# Use . instead of nothing here in this pattern because we already included
# the config in fvconf.mk and thus SRCDIR is set!
ifeq ($(HAVE_IPP),0)
  # We have no IPP, because of this we have to eliminate many filters
  # OBJS_libfvfilters = $(filter-out morphology/%,$(OBJS_libfvfilters))
  IPPI_FILTERS  = $(wildcard $(SRCDIR)/morphology/*.cpp)
  ALLFILES=$(realpath $(wildcard $(SRCDIR)/*.cpp $(SRCDIR)/*/*.cpp $(SRCDIR)/*/*/*.cpp))
  ifneq ($(ALLFILES),)
    IPPI_FILTERS += $(shell grep -rl ippi.h $(ALLFILES))
  endif
else
  LIBS_libfvfilters += ippi$(IPP_ARCH) ippcore$(IPP_ARCH) guide pthread
endif

ifeq ($(OBJSSUBMAKE),1)
.PHONY: print_unsupported
print_unsupported:
	$(SILENT)if [ "$(HAVE_IPP)" != "1" ]; then \
		for f in $(IPPI_FILTERS:$(SRCDIR)/%.cpp=%); do \
			echo -e "$(INDENT_PRINT)--- $(TRED)Omitting $$f filter$(TNORMAL) (No IPP installed)"; \
		done; \
	fi \

all: print_unsupported
endif

# We are lazy...
OBJS_libfvfilters := $(patsubst %.cpp,%.o,$(filter-out $(IPPI_FILTERS:$(SRCDIR)/%=%),$(subst $(SRCDIR)/,,$(realpath $(wildcard $(SRCDIR)/*.cpp $(SRCDIR)/*/*.cpp $(SRCDIR)/*/*/*.cpp)))))
LIBS_libfvfilters += fawkescore fawkesutils fvutils fvmodels

OBJS_all = $(OBJS_libfvfilters)
LIBS_all = $(LIBDIR)/libfvfilters.so


include $(BASEDIR)/etc/buildsys/base.mk

