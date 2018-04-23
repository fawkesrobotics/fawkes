#*****************************************************************************
#              Makefile Build System for Fawkes: MongoDB Plugin
#                            -------------------
#   Created on Sun Dec 05 23:15:27 2010 (Steelers vs. Baltimore)
#   Copyright (C) 2006-2010 by Tim Niemueller, AllemaniACs RoboCup Team
#
#*****************************************************************************
#
#   This program is free software; you can redistribute it and/or modify
#   it under the terms of the GNU General Public License as published by
#   the Free Software Foundation; either version 2 of the License, or
#   (at your option) any later version.
#
#*****************************************************************************

BASEDIR = ../../..

include $(BASEDIR)/etc/buildsys/config.mk
include $(LIBSRCDIR)/utils/utils.mk
include $(BUILDSYSDIR)/boost.mk

PRESUBDIRS = aspect interfaces

INCDIRS += $(INCDIRS_MONGODB)

MONGODB_REQ_BOOST_LIBS = system filesystem

LIBS_mongodb = fawkescore fawkesaspects fawkesmongodbaspect fawkesblackboard \
               MongoDBManagedReplicaSetInterface
OBJS_mongodb = mongodb_plugin.o mongodb_thread.o \
               mongodb_client_config.o \
               mongodb_instance_config.o \
               mongodb_replicaset_config.o

LIBS_mongodb_rrd = fawkescore fawkesutils fawkesaspects fawkesblackboard fawkesinterface \
                fawkesconfig fawkesmongodbaspect fawkesrrdaspect
OBJS_mongodb_rrd = mongorrd_plugin.o mongorrd_thread.o


CFLAGS  += $(CFLAGS_MONGODB) $(CFLAGS_CPP11)
LDFLAGS += $(LDFLAGS_MONGODB)

OBJS_all    = $(OBJS_mongodb) $(OBJS_mongodb_rrd)

HAVE_MONGODB_REQ_BOOST_LIBS = $(call boost-have-libs,$(MONGODB_REQ_BOOST_LIBS))
ifeq ($(HAVE_MONGODB_REQ_BOOST_LIBS),1)
  CFLAGS  += $(call boost-libs-cflags,$(MONGODB_REQ_BOOST_LIBS))
  LDFLAGS += $(call boost-libs-ldflags,$(MONGODB_REQ_BOOST_LIBS)) -lpthread
endif

ifeq ($(HAVE_MONGODB)$(HAVE_CPP14)$(HAVE_MONGODB_REQ_BOOST_LIBS),111)
  PLUGINS_all = $(PLUGINDIR)/mongodb.$(SOEXT) $(PLUGINDIR)/mongodb-rrd.$(SOEXT)
else
  ifneq ($(HAVE_MONGODB),1)
    WARN_TARGETS += warning_mongodb
  endif
  ifneq ($(HAVE_CPP14),1)
    WARN_TARGETS += warning_cpp14
  endif
  ifneq ($(HAVE_MONGODB_REQ_BOOST_LIBS),1)
    WARN_TARGETS_MONGODB_BOOST = $(foreach l,$(MONGODB_REQ_BOOST_LIBS),$(if $(call boost-have-lib,$l),, warning_mongodb_boost_$l))
	  WARN_TARGETS += $(WARN_TARGETS_MONGODB_BOOST)
  endif
endif

ifeq ($(OBJSSUBMAKE),1)
all: $(WARN_TARGETS)

.PHONY: warning_mongodb warning_cpp14
warning_mongodb:
	$(SILENT)echo -e "$(INDENT_PRINT)--> $(TRED)Omitting MongoDB Plugins$(TNORMAL) (mongodb[-devel] not installed)"
warning_cpp14:
	$(SILENT)echo -e "$(INDENT_PRINT)--> $(TRED)Omitting MongoDB Plugins$(TNORMAL) (C++ compiler too old, require at least C++14)"
$(WARN_TARGETS_MONGODB_BOOST): warning_mongodb_boost_%:
	$(SILENT)echo -e "$(INDENT_PRINT)--> $(TYELLOW)Omitting MongoDB plugin$(TNORMAL) (Boost library $* not found)"
endif

include $(BUILDSYSDIR)/base.mk

