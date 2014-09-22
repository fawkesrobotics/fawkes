#*****************************************************************************
#                Makefile Build System for Fawkes: protobuf bits
#                            -------------------
#   Created on Wed Jan 16 17:11:07 2013
#   Copyright (C) 2012 by Tim Niemueller, AllemaniACs RoboCup Team
#
#*****************************************************************************
#
#   This program is free software; you can redistribute it and/or modify
#   it under the terms of the GNU General Public License as published by
#   the Free Software Foundation; either version 2 of the License, or
#   (at your option) any later version.
#
#*****************************************************************************

ifndef __buildsys_config_mk_
$(error config.mk must be included before protobuf.mk)
endif

ifndef __buildsys_protobuf_mk_
__buildsys_protobuf_mk_ := 1

PROTOBUF_PROTOC = protoc

__PROTOBUF_INCLUDE_PATHS=/usr/include /usr/local/include

ifneq ($(PKGCONFIG),)
  HAVE_PROTOBUF_LIB = $(if $(shell $(PKGCONFIG) --exists 'protobuf'; echo $${?/1/}),1,0)
  HAVE_PROTOBUF_COMPLIB = $(if $(wildcard $(addsuffix /google/protobuf/compiler/importer.h,$(__PROTOBUF_INCLUDE_PATHS))),1)
  HAVE_PROTOBUF_COMP = $(if $(shell type -p $(PROTOBUF_PROTOC); echo $${?/1/}),1,0)
  ifeq ($(HAVE_PROTOBUF_LIB)$(HAVE_PROTOBUF_COMP)$(HAVE_PROTOBUF_COMPLIB),111)
    HAVE_PROTOBUF = 1
  endif
endif

ifeq ($(HAVE_PROTOBUF),1)
  CFLAGS_PROTOBUF  = -DHAVE_PROTOBUF $(shell $(PKGCONFIG) --cflags 'protobuf')
  LDFLAGS_PROTOBUF = $(shell $(PKGCONFIG) --libs 'protobuf')

  PROTOBUF_LIBDIR = $(LIBDIR)/protobuf
  LIBDIRS_BASE += $(PROTOBUF_LIBDIR)
endif

# Backward compatibility if protobuf.mk was included last
ifneq ($(PROTOBUF_all),)
  ifndef __buildsys_protobuf_msgs_mk_
    include $(BUILDSYSDIR)/protobuf_msgs.mk
  endif
endif

endif # __buildsys_protobuf_mk_
