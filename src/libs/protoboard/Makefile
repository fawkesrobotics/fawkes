#*****************************************************************************
#           Makefile Build System for Fawkes: protoboard Library
#                            -------------------
#   Created on Wed Jan 30 16:37:24 2013
#   Copyright (C) 2013 by Tim Niemueller, AllemaniACs RoboCup Team
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
include $(BUILDSYSDIR)/protobuf.mk
include $(BUILDSYSDIR)/boost.mk

CFLAGS   += $(CFLAGS_CPP11)

ifeq ($(CC),gcc)
  ifneq ($(call gcc_atleast_version,4,8),1)
    GCC_TOO_OLD=1
  endif
endif

LIBS_libfawkesprotoboard = \
  fawkescore fawkesaspects \
  fawkesblackboard fawkesinterface \
  fawkes_protobuf_comm \
  ProtobufPeerInterface

OBJS_libfawkesprotoboard = protobuf_thread.o protobuf_to_bb.o
 
LIBS_all = $(LIBDIR)/libfawkesprotoboard.$(SOEXT)
OBJS_all = $(OBJS_libfawkesprotoboard)


ifneq ($(GCC_TOO_OLD),1)
  LIBS_build  = $(LIBS_all)
else
  WARN_TARGETS += warning_old_gcc
endif

ALLOW_UNDEF_libfawkesprotoboard = 1

ifeq ($(OBJSSUBMAKE),1)
all: $(WARN_TARGETS) $(WARN_TARGETS_BOOST)
.PHONY: $(WARN_TARGETS)
warning_old_gcc:
	$(SILENT)echo -e "$(INDENT_PRINT)--> $(TRED)Can't build protoboard plugin$(TNORMAL) (GCC too old, have $(GCC_VERSION), required 4.8)"
endif

include $(BUILDSYSDIR)/base.mk

