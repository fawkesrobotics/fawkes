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

CFLAGS = -g

LIBS_qa_lua_context = fawkesutils fawkeslua
OBJS_qa_lua_context = qa_context.o

OBJS_all =	$(OBJS_qa_lua_context)
BINS_all =	$(BINDIR)/qa_lua_context
BINS_build = $(BINS_all)

include $(BUILDSYSDIR)/base.mk
