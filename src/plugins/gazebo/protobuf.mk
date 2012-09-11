#*****************************************************************************
#                 Makefile Build System for Fawkes: Protobuf bits
#                            -------------------
#   Created on Fri Aug 24 09:32:21 2012
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

ifndef __buildsys_config_mk_
$(error config.mk must be included before protobuf.mk)
endif

ifndef __buildsys_protobuf_mk_
__buildsys_protobuf_mk_ := 1


ifneq ($(PKGCONFIG),)
  HAVE_PROTOBUF = $(if $(shell $(PKGCONFIG) --atleast-version=2.4.0 'protobuf'; echo $${?/1/}),1,0)
else
  HAVE_PROTOBUF = 0
endif

ifeq ($(HAVE_PROTOBUF),1)
  CFLAGS_PROTOBUF  = -DHAVE_PROTOBUF $(shell $(PKGCONFIG) --cflags 'protobuf')
  LDFLAGS_PROTOBUF = $(shell $(PKGCONFIG) --libs 'protobuf')
endif

endif # __buildsys_protobuf_mk_

