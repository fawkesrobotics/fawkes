#*****************************************************************************
#            Makefile Build System for Fawkes : Interface Generator
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

BASEDIR = ../../../..

include $(BASEDIR)/etc/buildsys/config.mk
include $(BUILDSYSDIR)/interface.mk

ifeq ($(HAVE_INTERFACE_GENERATOR),1)
  CFLAGS += $(CFLAGS_LIBXMLPP) $(CFLAGS_LIBCRYPTO)
  CFLAGS += -Wno-mismatched-tags -Wno-inconsistent-missing-override
  LDFLAGS_ffifacegen = $(LDFLAGS_LIBXMLPP) $(LDFLAGS_LIBCRYPTO)
  LIBS_ffifacegen = stdc++ fawkescore fawkesutils

  OBJS_ffifacegen = constant.o cpp_generator.o digest.o		\
		    enum_constant.o field.o pseudomap.o main.o	\
		    message.o parser.o tolua_generator.o	\
		    checker.o

  OBJS_all     += $(OBJS_ffifacegen)
  BINS_all     += $(BINDIR)/ffifacegen
  MANPAGES_all += $(MANDIR)/man1/ffifacegen.1

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

include $(BUILDSYSDIR)/base.mk
