#*****************************************************************************
#            Makefile Build System for Fawkes : Interfaces Generator
#                            -------------------
#   Created on Fri Feb 01 11:37:25 2008
#   copyright (C) 2006-2008 by Tim Niemueller, AllemaniACs RoboCup Team
#
#*****************************************************************************
#
#   This program is free software; you can redistribute it and/or modify
#   it under the terms of the GNU General Public License as published by
#   the Free Software Foundation; either version 2 of the License, or
#   (at your option) any later version.
#
#*****************************************************************************

ifeq ($(BUILD_INTERFACE_GENERATOR),1)
  CFLAGS += -DINTERFACEDIR=\"$(realpath $(BASEDIR)/src/interfaces)\" \
	    $(shell $(PKGCONFIG) --cflags libxml++-2.6)

  ifneq ($(notdir $(SRCDIR)),generator)
    GENDIR=generator/
  endif

  LIBS_interface_generator = core utils
  LDFLAGS_interface_generator = $(shell $(PKGCONFIG) --libs libxml++-2.6)
  OBJS_interface_generator = $(GENDIR)constant.o		\
			     $(GENDIR)enum_constant.o	\
			     $(GENDIR)field.o		\
			     $(GENDIR)generator.o	\
			     $(GENDIR)main.o		\
			     $(GENDIR)message.o		\
			     $(GENDIR)parser.o		\
			     $(GENDIR)type_checker.o

  OBJS_all = $(OBJS_interface_generator)
  BINS_all = $(BINDIR)/interface_generator

else
  WARN_TARGETS += warning_libxmlpp
endif

ifeq ($(OBJSSUBMAKE),1)
  ifneq ($(WARN_TARGETS),)
all: $(WARN_TARGETS)
  endif
.PHONY: warning_libxmlpp
warning_libxmlpp:
	$(SILENT)echo -e "$(INDENT_PRINT)--> $(TRED)Omitting interface generator$(TNORMAL) (libxml++[-devel] not installed)"
	$(SILENT)echo -e "$(INDENT_PRINT)--> $(TYELLOW)Interfaces cannot be generated$(TNORMAL)"
endif

