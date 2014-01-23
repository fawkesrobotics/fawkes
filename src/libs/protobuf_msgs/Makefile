#*****************************************************************************
#           Makefile Build System for Fawkes: Configuration Utils
#                            -------------------
#   Created on Sun Jan 07 14:56:02 2007
#   Copyright (C) 2006-2007 by Tim Niemueller, AllemaniACs RoboCup Team
#			       Frederik Zwilling	
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
include $(BASEDIR)/src/plugins/gazebo/protobuf.mk

ifeq ($(HAVE_PROTOBUF),1)
  CFLAGS  += $(CFLAGS_PROTOBUF)
  LDFLAGS += $(LDFLAGS_PROTOBUF)
  LIBS_all  = $(LIBDIR)/libprotobufmsgs.so
else
  WARN_TARGETS += warning_protobuf
endif

ifeq ($(OBJSSUBMAKE),1)
all: $(WARN_TARGETS)

.PHONY: warning_protobuf
warning_protobuf:
	$(SILENT)echo -e "$(INDENT_PRINT)--> $(TRED)Omitting protobuf messages$(TNORMAL) (protobuf not found)"
endif

LIBS_libprotobufmsgs = stdc++
OBJS_libprotobufmsgs =$(filter-out $(FILTER_OUT) %_tolua.o,$(patsubst %.cpp,%.o,$(patsubst qa/%,,$(subst $(SRCDIR)/,,$(realpath $(wildcard $(SRCDIR)/*.cpp))))))
HDRS_libprotobufmsgs = $(subst $(SRCDIR)/,,$(wildcard $(SRCDIR)/*.h))

OBJS_all = $(OBJS_libprotobufmsgs)

include $(BUILDSYSDIR)/base.mk

