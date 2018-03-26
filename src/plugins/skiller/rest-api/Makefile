#*****************************************************************************
#      Makefile Build System for Fawkes: Behavior Engine REST API Plugin
#                            -------------------
#   Created on Sun Mar 25 01:36:22 2018
#   Copyright (C) 2006-2018 by Tim Niemueller
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
include $(BUILDSYSDIR)/rest-api.mk

LIBS_skiller_rest_api = \
	fawkescore fawkesutils fawkesaspects \
	fawkeswebview fawkesblackboard SkillerDebugInterface SkillerInterface
OBJS_skiller_rest_api = skiller-rest-api-plugin.o skiller-rest-api.o \
  $(patsubst %.cpp,%.o,$(subst $(SRCDIR)/,,$(realpath $(wildcard $(SRCDIR)/model/*.cpp))))

OBJS_all = $(OBJS_skiller_rest_api)

ifeq ($(HAVE_CPP17)$(HAVE_WEBVIEW)$(HAVE_RAPIDJSON),111)
  CFLAGS  += $(CFLAGS_WEBVIEW)  $(CFLAGS_RAPIDJSON)  $(CFLAGS_CPP17)
  LDFLAGS += $(LDFLAGS_WEBVIEW) $(LDFLAGS_RAPIDJSON) 

  PLUGINS_all = $(PLUGINDIR)/skiller-rest-api.$(SOEXT)
else
  ifneq ($(HAVE_CPP17),1)
    WARN_TARGETS += warning_cpp17
  endif
  ifneq ($(HAVE_WEBVIEW),1)
    WARN_TARGETS += warning_webview
  endif
  ifneq ($(HAVE_RAPIDJSON),1)
    WARN_TARGETS = warning_rapidjson
  endif
endif

ifeq ($(OBJSSUBMAKE),1)
all: $(WARN_TARGETS)
.PHONY: warning_webview warning_cpp17 warning_rapidjson
warning_webview:
	$(SILENT)echo -e "$(INDENT_PRINT)--> $(TRED)Cannot build skiller-rest-api plugin$(TNORMAL) (webview not available)"
warning_cpp17:
	$(SILENT)echo -e "$(INDENT_PRINT)--> $(TRED)Cannot build skiller-rest-api plugin$(TNORMAL) (C++17 not supported)"
warning_rapidjson:
	$(SILENT)echo -e "$(INDENT_PRINT)--> $(TRED)Cannot build skiller-rest-api plugin$(TNORMAL) (RapidJSON not found)"
endif

include $(BUILDSYSDIR)/base.mk
