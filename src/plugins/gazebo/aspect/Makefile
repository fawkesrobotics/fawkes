#*****************************************************************************
#                Makefile Build System for Fawkes: Gazebo Aspect
#                            -------------------
#   Created: Fri Aug 24 09:26:58 2012
#   Author  Bastian Klingen
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
include $(BUILDSYSDIR)/boost.mk
include $(BASEDIR)/src/plugins/gazebo/gazebo.mk
include $(BASEDIR)/src/plugins/gazebo/protobuf.mk

CFLAGS  += $(CFLAGS_GAZEBO) $(CFLAGS_PROTOBUF)
LDFLAGS += $(LDFLAGS_GAZEBO) $(LDFLAGS_PROTOBUF) -lm $(call boost-libs-ldflags,system)

LIBS_libfawkesgazeboaspect = stdc++ fawkescore fawkesaspects fawkesutils
OBJS_libfawkesgazeboaspect = $(patsubst %.cpp,%.o,$(patsubst qa/%,,$(subst $(SRCDIR)/,,$(realpath $(wildcard $(SRCDIR)/*.cpp)))))

OBJS_all    = $(OBJS_libfawkesgazeboaspect)

ifeq ($(HAVE_GAZEBO)$(HAVE_PROTOBUF)$(call boost-have-libs,system),111)
  LIBS_all = $(LIBDIR)/libfawkesgazeboaspect.so
else
  ifneq ($(HAVE_GAZEBO),1)
    WARN_TARGETS += warning_gazebo
  endif
  ifneq ($(HAVE_PROTOBUF),1)
    WARN_TARGETS += warning_protobuf
  endif
  ifneq ($(call boost-have-libs,system),1)
    WARN_TARGETS += warning_boost_system
  endif
endif

ifeq ($(OBJSSUBMAKE),1)
all: $(WARN_TARGETS)

.PHONY: warning_gazebo
warning_gazebo:
	$(SILENT)echo -e "$(INDENT_PRINT)--> $(TRED)Omitting Gazebo Aspect$(TNORMAL) (Gazebo not found)"

.PHONY: warning_protobuf
warning_protobuf:
	$(SILENT)echo -e "$(INDENT_PRINT)--> $(TRED)Omitting Gazebo Aspect$(TNORMAL) (Google Protobuf not found)"

.PHONY: warning_boost_system
warning_boost_system:
	$(SILENT)echo -e "$(INDENT_PRINT)--> $(TRED)Omitting Gazebo Aspect$(TNORMAL) (Boost System not found)"
endif

include $(BUILDSYSDIR)/base.mk

