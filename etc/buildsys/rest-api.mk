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
RESTAPI_OUTDIR=$(SRCDIR)/model
RESTAPI_SPECFILES=api.yaml
RESTAPI_TEMPLATE_DIR=$(RESTAPI_BASEDIR)/templates
RESTAPI_TEMPLATES=c++17.h.model.template c++17.cpp.model.template
RESTAPI_FORCE=-f
RESTAPI_STAMPFILES=$(addsuffix .stamp,$(RESTAPI_SPECFILES))

ifneq ($(filter restapi-models,$(MAKECMDGOALS)),)
restapi-models: $(addprefix $(SRCDIR)/$(OBJDIR)/,$(RESTAPI_STAMPFILES))
endif

$(SRCDIR)/$(OBJDIR)/%.stamp: $(SRCDIR)/% $(addprefix $(RESTAPI_TEMPLATE_DIR)/,$(RESTAPI_TEMPLATES))
	$(SILENT) echo -e "$(INDENT_PRINT)[RestAPI] $(PARENTDIR)$(TBOLDGRAY)$(<F)$(TNORMAL)"
	$(SILENT)$(RESTAPI_APIGEN) $(RESTAPI_FORCE) \
		--api $< --output-dir $(RESTAPI_OUTDIR) \
		--template-dir $(RESTAPI_TEMPLATE_DIR) $(RESTAPI_TEMPLATES) | \
		sed -e "s|^$(realpath $(BASEDIR))/\(.*/\)\([^/]\+\)$$|$(INDENT_PRINT)[RestAPI] -> \1$(patsubst \\%,\\o%,$(TBOLDGRAY))\2$(patsubst \\%,\\o%,$(TNORMAL))|g"
	$(SILENT)touch $@

ifeq ($(OBJSSUBMAKE),1)
.PHONY: warning_tolua_wrapper
warning_tolua_wrapper:
	$(SILENT)echo -e "$(INDENT_PRINT)[WARN] $(TRED)Omitting Lua compatibility wrapper in $(PARENTDIR)$(TNORMAL) (tolua++[-devel] not installed)"
endif # OBJSSUBMAKE is 1

endif # __buildsys_rest_api_mk_

