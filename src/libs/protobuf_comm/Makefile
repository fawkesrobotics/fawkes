#*****************************************************************************
#           Makefile Build System for Fawkes: protobuf_comm Library
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

REQ_BOOST_LIBS = thread asio system signals2
HAVE_BOOST_LIBS = $(call boost-have-libs,$(REQ_BOOST_LIBS))

ifneq ($(PKGCONFIG),)
  HAVE_LIBCRYPTO := $(if $(shell $(PKGCONFIG) --exists 'libcrypto'; echo $${?/1/}),1,0)
  LIBCRYPTO_PKG  := libcrypto
  ifneq ($(HAVE_LIBCRYPTO),1)
    HAVE_LIBCRYPTO := $(if $(shell $(PKGCONFIG) --exists 'openssl'; echo $${?/1/}),1,0)
    LIBCRYPTO_PKG  := openssl
  endif
endif
ifeq ($(HAVE_LIBCRYPTO),1)
  CFLAGS_LIBCRYPTO  += -DHAVE_LIBCRYPTO $(shell $(PKGCONFIG) --cflags $(LIBCRYPTO_PKG))
  LDFLAGS_LIBCRYPTO += $(shell $(PKGCONFIG) --libs $(LIBCRYPTO_PKG))
endif

LIBS_libfawkes_protobuf_comm = stdc++ m
OBJS_libfawkes_protobuf_comm = $(patsubst %.cpp,%.o,$(patsubst qa/%,,$(subst $(SRCDIR)/,,$(realpath $(wildcard $(SRCDIR)/*.cpp $(SRCDIR)/*/*.cpp $(SRCDIR)/*/*/*.cpp)))))
HDRS_libfawkes_protobuf_comm = $(subst $(SRCDIR)/,,$(wildcard $(SRCDIR)/*.h $(SRCDIR)/*/*.h $(SRCDIR)/*/*/*.h))

OBJS_all = $(OBJS_libfawkes_protobuf_comm)

ifeq ($(HAVE_CPP11)$(HAVE_PROTOBUF)$(HAVE_BOOST_LIBS),111)
  CFLAGS  += $(CFLAGS_PROTOBUF) $(call boost-libs-cflags,$(REQ_BOOST_LIBS)) $(CFLAGS_CPP11) $(CFLAGS_LIBCRYPTO)
  LDFLAGS += $(LDFLAGS_PROTOBUF) $(call boost-libs-ldflags,$(REQ_BOOST_LIBS)) $(LDFLAGS_LIBCRYPTO)

  LIBS_all  = $(LIBDIR)/libfawkes_protobuf_comm.so
else
  ifneq ($(HAVE_PROTOBUF),1)
    WARN_TARGETS += warning_protobuf
  endif
  ifneq ($(HAVE_BOOST_LIBS),1)
    WARN_TARGETS_BOOST = $(foreach l,$(REQ_BOOST_LIBS),$(if $(call boost-have-lib,$l),, warning_boost_$l))
  endif
endif

ifeq ($(OBJSSUBMAKE),1)
all: $(WARN_TARGETS) $(WARN_TARGETS_BOOST)
.PHONY: $(WARN_TARGETS)
warning_protobuf:
	$(SILENT)echo -e "$(INDENT_PRINT)--> $(TRED)Cannot build protobuf_comm library$(TNORMAL) (protobuf not found)"

$(WARN_TARGETS_BOOST): warning_boost_%:
	$(SILENT)echo -e "$(INDENT_PRINT)--> $(TRED)Cannot build protobuf_comm library$(TNORMAL) (Boost library $* not found)"

endif

include $(BUILDSYSDIR)/base.mk

