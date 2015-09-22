#*****************************************************************************
#             Makefile Build System for Fawkes: protobuf message bits
#                            -------------------
#   Created on Wed Mar 05 14:41:36 2014
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

ifndef __buildsys_protobuf_mk_
  include $(BUILDSYSDIR)/protobuf.mk
endif

ifndef __buildsys_protobuf_msgs_mk_
__buildsys_protobuf_msgs_mk_ := 1

ifeq ($(HAVE_PROTOBUF),1)
  ifneq ($(PROTOBUF_all),)
    $(foreach P,$(PROTOBUF_all),							\
	$(if $(filter-out %_msgs,$P),						\
	$(eval LDFLAGS_protobuf_lib$P     = $(LDFLAGS_PROTOBUF) -lfawkesutils)	\
	$(eval OBJS_protobuf_lib$P        = $P_ctor.o $P.pb.o)			\
	$(eval HDRS_interfaces_lib$P      = $P.pb.h)				\
	$(eval INST_LIB_SUBDIR_protobuf_lib$P   = $(FFLIBSUBDIR))		\
	$(eval INST_HDRS_SUBDIR_protobuf_lib$P  = protobuf)			\
	$(eval OBJS_all                  += $$(OBJS_protobuf_lib$P))		\
	$(eval PROTOBUF_CTORS            += $(SRCDIR)/$P_ctor.cpp)		\
	$(eval PROTOBUF_SRCS             += $(SRCDIR)/$P.pb.cpp)		\
	$(eval PROTOBUF_HDRS             += $(SRCDIR)/$P.pb.h)			\
	$(eval PROTOBUF_LIBS             += $(PROTOBUF_LIBDIR)/lib$P.$(SOEXT))	\
	$(eval LIBS_all                  += $(PROTOBUF_LIBDIR)/lib$P.$(SOEXT))	\
	$(eval CLEAN_FILES               += $P.pb.h $P.pb.cpp)			\
	, \
	$(eval LDFLAGS_protobuf_lib$P     = $(LDFLAGS_PROTOBUF) -lfawkesutils)	\
	$(eval OBJS_protobuf_lib$P        = $P_ctor.o $(MSGS_$P:%=%.pb.o))	\
	$(eval HDRS_interfaces_lib$P      = $(MSGS_$P:%=%.pb.h))		\
	$(eval INST_LIB_SUBDIR_protobuf_lib$P   = $(FFLIBSUBDIR))		\
	$(eval INST_HDRS_SUBDIR_protobuf_lib$P  = protobuf)			\
	$(eval OBJS_all                  += $$(OBJS_protobuf_lib$P))		\
	$(eval PROTOBUF_SRCS             += $(MSGS_$P:%=$(SRCDIR)/%.pb.cpp))	\
	$(eval PROTOBUF_HDRS             += $(MSGS_$P:%=$(SRCDIR)/%.pb.h))	\
	$(eval PROTOBUF_LIBS             += $(PROTOBUF_LIBDIR)/lib$P.$(SOEXT))	\
	$(eval OBJS_lib$P.$(SOEXT)	  = $(MSGS_$P:%=%.pb.o))		\
	$(eval LIBS_all                  += $(PROTOBUF_LIBDIR)/lib$P.$(SOEXT))	\
	$(eval CLEAN_FILES               += $(MSGS_$P:%=%.pb.h) 		\
					    $(MSGS_$P:%=%.pb.cpp))		\
	)\
  )
  endif

define PROTOBUF_LIB_CTOR =
#include <utils/system/dynamic_module/module.h>
void pb_prevent_unloading(void) __attribute__((constructor));
void pb_prevent_unloading(void)
{
  fawkes::Module m(LIBDIR"/protobuf/@LIBNAME@.$(SOEXT)",
                   fawkes::Module::MODULE_BIND_LAZY | fawkes::Module::MODULE_NODELETE);
  m.open();
}
endef
export PROTOBUF_LIB_CTOR

ifeq ($(OBJSSUBMAKE),1)

$(PROTOBUF_SRCS): $(SRCDIR)/%.pb.cpp: $(SRCDIR)/$(OBJDIR)/%.pb.touch
$(PROTOBUF_HDRS): $(SRCDIR)/%.pb.h: $(SRCDIR)/$(OBJDIR)/%.pb.touch

%_ctor.o:
	$(SILENT)echo "$$PROTOBUF_LIB_CTOR" | sed -e 's/@LIBNAME@/lib$*/' | \
		$(CC) -x c++ -o $@ $(CFLAGS_BASE)$(addprefix -I,$(INCDIRS)) -c -

$(SRCDIR)/$(OBJDIR)/%.pb.touch: $(SRCDIR)/%.proto
	$(SILENT) echo -e "$(INDENT_PRINT)[PRB] $(PARENTDIR)$(TBOLDGRAY)$(<F)$(TNORMAL)"
	$(SILENT)$(PROTOBUF_PROTOC) --cpp_out $(SRCDIR) --proto_path $(SRCDIR) $<
	$(SILENT) mv $(SRCDIR)/$*.pb.cc $(SRCDIR)/$*.pb.cpp
	$(SILENT) mkdir -p $(@D)
	$(SILENT) touch $@

.SECONDARY: $(PROTOBUF_SRCS) $(PROTOBUF_HDRS)

endif # OBJSSUBMAKE == 1

$(PROTOBUF_LIBDIR)/lib%_msgs.so: $$(patsubst %.proto,%.pb.o,$$(MSGS_$$(call nametr,$$*)))
	$(SILENT)echo $@ from $@


ifneq ($(PLUGINS_all),)
$(PLUGINS_all:%.$(SOEXT)=%.$(SOEXT)): | $(PROTOBUF_LIBS:%.$(SOEXT)=%.$(SOEXT))
endif
ifneq ($(BINS_all),)
$(BINS_all): | $(PROTOBUF_LIBS)
endif

else # HAVE_PROTOBUF != 1
  ifneq ($(PROTOBUF_all),)
    WARN_TARGETS += warning_protobuf_msgs
  endif

  ifeq ($(OBJSSUBMAKE),1)
all: $(WARN_TARGETS)
.PHONY: warning_protobuf_msgs
warning_protobuf_msgs:
	$(SILENT)echo -e "$(INDENT_PRINT)--> $(TRED)Cannot build protobuf messages$(TNORMAL) (protobuf not found)"
  endif

endif # HAVE_PROTOBUF

endif # __buildsys_protobuf_msgs_mk_
