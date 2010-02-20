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

include $(BASEDIR)/etc/buildsys/interface.mk

ifeq ($(HAVE_INTERFACE_GENERATOR),1)
  CFLAGS += $(shell $(PKGCONFIG) --cflags libxml++-2.6)

  ifneq ($(notdir $(SRCDIR)),generator)
    GENDIR=generator/
  endif

  LIBS_ffifacegen = fawkescore fawkesutils
  LDFLAGS_ffifacegen = $(shell $(PKGCONFIG) --libs libxml++-2.6) $(LDFLAGS_LIBCRYPTO)
  CFLAGS += $(CFLAGS_LIBXMLPP) $(CFLAGS_LIBCRYPTO)
  OBJS_ffifacegen = $(GENDIR)constant.o	\
		    $(GENDIR)cpp_generator.o	\
		    $(GENDIR)digest.o		\
		    $(GENDIR)enum_constant.o	\
		    $(GENDIR)field.o		\
		    $(GENDIR)pseudomap.o	\
		    $(GENDIR)main.o		\
		    $(GENDIR)message.o		\
		    $(GENDIR)parser.o		\
		    $(GENDIR)tolua_generator.o	\
		    $(GENDIR)type_checker.o

  OBJS_all += $(OBJS_ffifacegen)
  BINS_all += $(BINDIR)/ffifacegen

else
  ifneq ($(HAVE_LIBXMLPP),1)
    WARN_TARGETS += warning_libxmlpp
  endif
  ifneq ($(HAVE_LIBCRYPTO),1)
    WARN_TARGETS += warning_openssl
  endif
endif

ifeq ($(OBJSSUBMAKE),1)
  ifneq ($(WARN_TARGETS),)
all: $(WARN_TARGETS)
  endif
.PHONY: warning_libxmlpp
warning_libxmlpp:
	$(SILENT)echo -e "$(INDENT_PRINT)--> $(TRED)Omitting interface generator$(TNORMAL) (libxml++[-devel] not installed)"
	$(SILENT)echo -e "$(INDENT_PRINT)--> $(TYELLOW)Interfaces cannot be generated$(TNORMAL)"

.PHONY: warning_openssl
warning_openssl:
	$(SILENT)echo -e "$(INDENT_PRINT)--> $(TRED)Omitting interface generator$(TNORMAL) (openssl[-devel] not installed)"
	$(SILENT)echo -e "$(INDENT_PRINT)--> $(TYELLOW)Interfaces cannot be generated$(TNORMAL)"
endif

