#*****************************************************************************
#             Makefile Build System for Fawkes: REST API bits
#                            -------------------
#   Created on Thu Mar 22 12:47:02 2018
#   Copyright (C) 2006-2018 by Tim Niemueller, AllemaniACs RoboCup Team
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
$(error config.mk must be included before rest-api.mk)
endif

ifndef __buildsys_rest_api_mk_
__buildsys_rest_api_mk_ := 1

include $(BUILDCONFDIR)/webview/webview.mk

ifeq ($(HAVE_CPP17)$(HAVE_RAPIDJSON),11)
  HAVE_RESTAPI=1
endif

RESTAPI_BASEDIR=$(FAWKES_BASEDIR)/etc/restapi-gen
RESTAPI_APIGEN=$(RESTAPI_BASEDIR)/restapi-gen.py
RESTAPI_OUTDIR_MODELS=$(SRCDIR)/model
RESTAPI_OUTDIR_API=$(SRCDIR)/api
RESTAPI_SPECFILES=api.yaml
RESTAPI_TEMPLATE_DIR=$(RESTAPI_BASEDIR)/templates
RESTAPI_TEMPLATES_CPP=c++17.h.model.template c++17.cpp.model.template
RESTAPI_TEMPLATES_TS_MODELS=typescript-angular.ts.model.template
RESTAPI_TEMPLATES_TS_API=typescript-angular.ts.api.template
RESTAPI_FORCE=-f
RESTAPI_STAMPFILES_CPP=$(addsuffix .cpp.stamp,$(RESTAPI_SPECFILES))
RESTAPI_STAMPFILES_TYPESCRIPT=$(addsuffix .ts.stamp,$(RESTAPI_SPECFILES))

WEBVIEW_FRONTEND_DIR=$(FAWKES_BASEDIR)/src/plugins/webview/frontend
WEBVIEW_FRONTEND_SRCDIR=$(WEBVIEW_FRONTEND_DIR)/src

ifneq ($(RESTAPI_STAMPFILES_CPP)$(RESTAPI_STAMPFILES_TYPESCRIPT),)
restapi-models: $(addprefix $(SRCDIR)/$(OBJDIR)/,$(RESTAPI_STAMPFILES_CPP))
restapi-cpp: restapi-models ; @:
restapi-webview: $(addprefix $(SRCDIR)/$(OBJDIR)/,$(RESTAPI_STAMPFILES_TYPESCRIPT))
endif

$(SRCDIR)/$(OBJDIR)/%.cpp.stamp: $(SRCDIR)/% $(addprefix $(RESTAPI_TEMPLATE_DIR)/,$(RESTAPI_TEMPLATES_CPP))
	$(SILENT) echo -e "$(INDENT_PRINT)[RestAPI/C++] $(PARENTDIR)$(TBOLDGRAY)$(<F)$(TNORMAL)"
	$(SILENT)$(RESTAPI_APIGEN) $(RESTAPI_FORCE) \
		--api $< --output-dir $(RESTAPI_OUTDIR_MODELS) \
		--template-dir $(RESTAPI_TEMPLATE_DIR) $(RESTAPI_TEMPLATES_CPP) | \
		sed -e "s|^$(realpath $(BASEDIR))/\(.*/\)\([^/]\+\)$$|$(INDENT_PRINT)[RestAPI/C++] -> \1$(patsubst \\%,\\o%,$(TBOLDGRAY))\2$(patsubst \\%,\\o%,$(TNORMAL))|g"
	$(SILENT)touch $@

$(SRCDIR)/$(OBJDIR)/%.ts.stamp: $(SRCDIR)/% $(addprefix $(RESTAPI_TEMPLATE_DIR)/,$(RESTAPI_TEMPLATES_TS_MODELS) $(RESTAPI_TEMPLATES_TS_API))
	$(SILENT) echo -e "$(INDENT_PRINT)[RestAPI/TS/A] $(PARENTDIR)$(TBOLDGRAY)$(<F)$(TNORMAL)"
	$(SILENT) mkdir -p $(RESTAPI_OUTDIR_MODELS)
	$(SILENT)$(RESTAPI_APIGEN) $(RESTAPI_FORCE) \
		--api $< --output-dir $(RESTAPI_OUTDIR_MODELS) \
		--template-dir $(RESTAPI_TEMPLATE_DIR) $(RESTAPI_TEMPLATES_TS_MODELS) | \
	sed -e "s|^$(realpath $(BASEDIR))/\(.*/\)\([^/]\+\)$$|$(INDENT_PRINT)[RestAPI/TS/A] -> \1$(patsubst \\%,\\o%,$(TBOLDGRAY))\2$(patsubst \\%,\\o%,$(TNORMAL))|g"
	$(SILENT) mkdir -p $(RESTAPI_OUTDIR_API)
	$(SILENT)$(RESTAPI_APIGEN) $(RESTAPI_FORCE) \
		--api $< --output-dir $(RESTAPI_OUTDIR_API) \
		--template-dir $(RESTAPI_TEMPLATE_DIR) $(RESTAPI_TEMPLATES_TS_API) | \
	sed -e "s|^$(realpath $(BASEDIR))/\(.*/\)\([^/]\+\)$$|$(INDENT_PRINT)[RestAPI/TS/A] -> \1$(patsubst \\%,\\o%,$(TBOLDGRAY))\2$(patsubst \\%,\\o%,$(TNORMAL))|g"
	$(SILENT)touch $@

ifeq ($(OBJSSUBMAKE),1)
.PHONY: warning_tolua_wrapper
warning_tolua_wrapper:
	$(SILENT)echo -e "$(INDENT_PRINT)[WARN] $(TRED)Omitting Lua compatibility wrapper in $(PARENTDIR)$(TNORMAL) (tolua++[-devel] not installed)"
endif # OBJSSUBMAKE is 1

endif # __buildsys_rest_api_mk_

