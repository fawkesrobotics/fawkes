#*****************************************************************************
#           Makefile Build System for Fawkes: protobuf_clips Library
#                            -------------------
#   Created on Tue Apr 16 13:38:13 2013
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
include $(BUILDSYSDIR)/clips.mk

REQ_BOOST_LIBS = system
HAVE_BOOST_LIBS = $(call boost-have-libs,$(REQ_BOOST_LIBS))

LIBS_libfawkes_protobuf_clips = stdc++ m fawkescore fawkeslogging
OBJS_libfawkes_protobuf_clips = $(patsubst %.cpp,%.o,$(patsubst qa/%,,$(subst $(SRCDIR)/,,$(realpath $(wildcard $(SRCDIR)/*.cpp)))))
HDRS_libfawkes_protobuf_clips = $(subst $(SRCDIR)/,,$(wildcard $(SRCDIR)/*.h))

OBJS_all = $(OBJS_libfawkes_protobuf_clips)
LIBS_all = $(LIBDIR)/libfawkes_protobuf_clips.so

ifeq ($(HAVE_CPP11)$(HAVE_PROTOBUF)$(HAVE_PROTOBUF_COMM)$(HAVE_CLIPS)$(HAVE_BOOST_LIBS),11111)
  CFLAGS  += $(CFLAGS_PROTOBUF) $(CFLAGS_PROTOBUF_COMM) $(CFLAGS_CLIPS) $(CFLAGS_CPP11) \
	     $(call boost-libs-cflags,$(REQ_BOOST_LIBS))
  LDFLAGS += $(LDFLAGS_PROTOBUF) $(LDFLAGS_PROTOBUF_COMM) $(LDFLAGS_CLIPS) \
	     $(call boost-libs-ldflags,$(REQ_BOOST_LIBS))

  LIBS_build = $(LIBS_all)
else
  ifneq ($(HAVE_CPP11),1)
    WARN_TARGETS += warning_cpp11
  endif
  ifneq ($(HAVE_PROTOBUF),1)
    WARN_TARGETS += warning_protobuf
  endif
  ifneq ($(HAVE_PROTOBUF_COMM),1)
    WARN_TARGETS += warning_protobuf_comm
  endif
  ifneq ($(HAVE_CLIPS),1)
    WARN_TARGETS += warning_clips
  endif
  ifneq ($(HAVE_BOOST_LIBS),1)
    WARN_TARGETS_BOOST = $(foreach l,$(REQ_BOOST_LIBS),$(if $(call boost-have-lib,$l),, warning_boost_$l))
  endif
endif

ifeq ($(OBJSSUBMAKE),1)
all: $(WARN_TARGETS)  $(WARN_TARGETS_BOOST)
.PHONY: $(WARN_TARGETS)
warning_protobuf:
	$(SILENT)echo -e "$(INDENT_PRINT)--> $(TRED)Cannot build protobuf_clips library$(TNORMAL) (protobuf not found)"

warning_protobuf_comm:
	$(SILENT)echo -e "$(INDENT_PRINT)--> $(TRED)Cannot build protobuf_clips library$(TNORMAL) (protobuf_comm not found)"

warning_clips:
	$(SILENT)echo -e "$(INDENT_PRINT)--> $(TRED)Cannot build protobuf_clips library$(TNORMAL) ($(CLIPS_ERROR))"

warning_cpp11:
	$(SILENT)echo -e "$(INDENT_PRINT)--> $(TRED)Cannot build protobuf_clips library$(TNORMAL) (C++11 not supported)"

$(WARN_TARGETS_BOOST): warning_boost_%:
	$(SILENT)echo -e "$(INDENT_PRINT)--> $(TRED)Cannot build protobuf_comm library$(TNORMAL) (Boost library $* not found)"
endif

include $(BUILDSYSDIR)/base.mk

