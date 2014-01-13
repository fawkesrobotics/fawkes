#*****************************************************************************
#        Makefile Build System for Fawkes : FireVision Stereo Utils
#                            -------------------
#   Created on Mon Jun 11 16:12:01 2007
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

ifneq ($(HAVE_TRICLOPS),1)
  WARN_TARGETS   += warning_triclops
endif
ifeq ($(HAVE_TRICLOPS),1)
  CFLAGS  += $(CFLAGS_TRICLOPS)
  LDFLAGS += $(LDFLAGS_TRICLOPS)
  LIBS    += m fvcams
else
  FILTER_OUT   += triclops.o
endif

# Use . instead of nothing here in this pattern because we already included
# the config in fvconf.mk and thus SRCDIR is set!
ifeq ($(OBJSSUBMAKE),1)
all: $(WARN_TARGETS)
warning_triclops:
	$(SILENT)echo -e "$(INDENT_PRINT)--- $(TRED)Omitting Triclops SDK support$(TNORMAL) ($(TRICLOPS_SDK_ERR))"
endif

# We are lazy in the utils...
OBJS_libfvstereo := $(filter-out $(FILTER_OUT),$(patsubst %.cpp,%.o,$(patsubst qa/%,,$(subst $(SRCDIR)/,,$(realpath $(wildcard $(SRCDIR)/*.cpp $(SRCDIR)/*/*.cpp $(SRCDIR)/*/*/*.cpp))))))
LIBS_libfvstereo := fawkescore fawkesutils fvutils fvcams
HDRS_libfvstereo  = $(patsubst %.o,%.h,$(OBJS_libfvstereo))

OBJS_all = $(OBJS_libfvstereo)
LIBS_all = $(LIBDIR)/libfvstereo.so

include $(BUILDSYSDIR)/base.mk

triclops:
	$(SILENT)if [ -e "$(TRICLOPS_SDK)/include/triclops.h" -a -e "$(TRICLOPS_SDK)/lib/libtriclops.a" -a -e "$(TRICLOPS_SDK)/lib/libpnmutils.a" ]; then \
		echo -e "$(INDENT_PRINT)--> Creating $(TBOLDGREEN)$(TRICLOPS_SDK)/lib/libtriclops.so$(TNORMAL)"; \
		cd "$(TRICLOPS_SDK)/lib"; \
		rm -f libtriclops.so; \
		rm -rf objs; \
		mkdir objs; \
		cd objs; \
		ar x ../libtriclops.a; \
		ar x ../libpnmutils.a; \
		cd ..; \
		$(CC) -shared -m32 -lstdc++ -o libtriclops.so objs/*; \
	else \
		echo -e "$(INDENT_PRINT)$(TRED)--> Cannot create $(TRICLOPS_SDK)/lib/libtriclops.so$(TNORMAL), Triclops SDK not found or incomplete.$(TNORMAL)"; \
	fi

