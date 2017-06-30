#*****************************************************************************
#               Makefile Build System for Fawkes: kdl_parser
#                            -------------------
#   Created on Feb 14, 2014
#   Copyright (C) 2006-2014
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
include $(BUILDSYSDIR)/boost.mk
include $(BUILDSYSDIR)/eigen3.mk

LIBS_libfawkeskdl_parser = fawkescore
OBJS_libfawkeskdl_parser = $(patsubst %.cpp,%.o,$(patsubst qa/%,,$(subst $(SRCDIR)/,,$(realpath $(wildcard $(SRCDIR)/*.cpp)))))
HDRS_libfawkeskdl_parser = $(subst $(SRCDIR)/,,$(wildcard $(SRCDIR)/*.h))

OBJS_all = $(OBJS_libfawkeskdl_parser)
ifeq ($(HAVE_KDLPARSER)$(HAVE_EIGEN3),11)
  CFLAGS  += $(CFLAGS_KDLPARSER) $(CFLAGS_EIGEN3)
  ifeq ($(HAVE_CPP11),1)
    CFLAGS += $(CFLAGS_CPP11)
  endif
  LDFLAGS += $(LDFLAGS_KDLPARSER) $(LDFLAGS_EIGEN3)
  LIBS_all = $(LIBDIR)/libfawkeskdl_parser.so
else
  ifneq ($(HAVE_KDL),1)
    WARN_TARGETS += warning_kdl
  endif
  ifneq ($(HAVE_URDFDOMHEADERS),1)
    WARN_TARGETS += warning_urdfdomheaders
  endif
  ifneq ($(HAVE_URDFDOM),1)
    WARN_TARGETS += warning_urdfdom
  endif
  ifneq ($(HAVE_TINYXML),1)
    WARN_TARGETS += warning_tinyxml
  endif
endif

ifeq ($(OBJSSUBMAKE),1)
all: $(WARN_TARGETS)
.PHONY: warning_kdl warning_urdfdomheaders warning_urdfdom warning_tinyxml
warning_kdl:
	$(SILENT)echo -e "$(INDENT_PRINT)--> $(TRED)Omitting KDL parser library$(TNORMAL) (orocos-kdl[-devel] not available)"
warning_urdfdomheaders:
	$(SILENT)echo -e "$(INDENT_PRINT)--> $(TRED)Omitting KDL parser library$(TNORMAL) (urdfdom-headers[-devel] not available)"
warning_urdfdom:
	$(SILENT)echo -e "$(INDENT_PRINT)--> $(TRED)Omitting KDL parser library$(TNORMAL) (urdfdom[-devel] not available)"
warning_tinyxml:
	$(SILENT)echo -e "$(INDENT_PRINT)--> $(TRED)Omitting KDL parser library$(TNORMAL) (tinyxml not available)"
endif

include $(BUILDSYSDIR)/base.mk
	
