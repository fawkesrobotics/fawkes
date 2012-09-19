#*****************************************************************************
#            Makefile Build System for Fawkes: Nao Plugins
#                            -------------------
#   Created on Thu May 12 15:08:59 2011
#   Copyright (C) 2006-2011 by Tim Niemueller, AllemaniACs RoboCup Team
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

PRESUBDIRS = interfaces
SUBDIRS = naogui

LIBS_naoqi = m fawkescore fawkesutils fawkesaspects fawkesblackboard \
	   fawkesinterface fawkesnaoqiaspect fawkesnaoqibroker \
	   fawkesnaoqi_dcm_utils fawkesnaoqi_motion_utils \
	   alcore alcommon alvalue almemoryfastaccess \
	   boost_serialization-mt boost_signals-mt \
	   NaoJointPositionInterface NaoJointStiffnessInterface \
	   NaoSensorInterface
OBJS_naoqi = naoqi_plugin.o broker_thread.o dcm_thread.o

LIBS_naoqi_motion = fawkescore fawkesutils fawkesaspects fawkesblackboard \
		    fawkesinterface fawkesnaoqiaspect fawkesnaoqibroker \
		    alcore alcommon alvalue althread \
		    fawkesnaoqi_motion_utils \
		    boost_serialization-mt boost_signals-mt \
		    HumanoidMotionInterface NaoSensorInterface
OBJS_naoqi_motion = motion_plugin.o motion_thread.o \
		    motion_kick_task.o motion_standup_task.o

LIBS_naoqi_speechsynth = fawkescore fawkesutils fawkesaspects fawkesblackboard \
			 fawkesinterface fawkesnaoqiaspect fawkesnaoqibroker \
			 alcore alcommon alvalue \
			 boost_serialization-mt boost_signals-mt \
			 SpeechSynthInterface
OBJS_naoqi_speechsynth = speechsynth_plugin.o speechsynth_thread.o

LIBS_naoqi_leds = m fawkescore fawkesutils fawkesaspects fawkesblackboard \
		  fawkesinterface fawkesnaoqiaspect fawkesnaoqibroker \
		  fawkesnaoqi_dcm_utils \
		  alcore alcommon alvalue almemoryfastaccess \
		  boost_serialization-mt boost_signals-mt \
		  LedInterface NaoJointPositionInterface
OBJS_naoqi_leds = led_plugin.o led_thread.o

LIBS_naoqi_buttons = m fawkescore fawkesutils fawkesaspects fawkesblackboard \
		  fawkesinterface fawkesnaoqiaspect fawkesnaoqibroker \
		  fawkesnaoqi_dcm_utils \
		  alcore alcommon alvalue almemoryfastaccess \
		  boost_serialization-mt boost_signals-mt \
		  SwitchInterface NaoSensorInterface
OBJS_naoqi_buttons = button_plugin.o button_thread.o

LIBS_libnaofawkes = dl fawkescore fawkesutils fawkesaspects fawkesblackboard \
		    fawkesinterface fawkesnaoqiaspect fawkesbaseapp \
		    fawkesplugin fawkesnaoqibroker\
		    alcore alcommon alvalue \
		    boost_serialization-mt
OBJS_libnaofawkes = naofawkes_module.o

LIBS_libfawkesnaoqibroker = alcore alcommon
OBJS_libfawkesnaoqibroker = naoqi_broker.o

LIBS_libfawkesnaoqi_dcm_utils = alcore alcommon alvalue
OBJS_libfawkesnaoqi_dcm_utils = dcm_utils.o

LIBS_libfawkesnaoqi_motion_utils = alcore alcommon alvalue
OBJS_libfawkesnaoqi_motion_utils = motion_utils.o

OBJS_all = $(OBJS_naoqi) $(OBJS_libnaofawkes) $(OBJS_libfawkesnaoqibroker) \
	$(OBJS_libfawkesnaoqi_dcm_utils) $(OBJS_libfawkesnaoqi_motion_utils)\
	$(OBJS_naoqi_motion) $(OBJS_naoqi_speechsynth) $(OBJS_naoqi_leds) \
	$(OBJS_naoqi_buttons)

ifeq ($(HAVE_NAOQI),1)
  PRESUBDIRS += aspect

  CFLAGS  += $(CFLAGS_NAOQI)
  LDFLAGS += $(LDFLAGS_NAOQI)
  PLUGINS_all = $(PLUGINDIR)/naoqi.so $(PLUGINDIR)/naoqi-motion.so \
		$(PLUGINDIR)/naoqi-speechsynth.so \
		$(PLUGINDIR)/naoqi-leds.so \
		$(PLUGINDIR)/naoqi-buttons.so
  LIBS_all = $(LIBDIR)/libfawkesnaoqibroker.so \
	     $(LIBDIR)/libfawkesnaoqi_dcm_utils.so \
	     $(LIBDIR)/libfawkesnaoqi_motion_utils.so \
	     $(LIBDIR)/libnaofawkes.so

  TARGETS_all = info_naoqiroot
else
  WARN_TARGETS += warning_naoqi
endif

# Ordering constraints
$(LIBDIR)/libnaofawkes.so $(PLUGINS_all): | $(LIBDIR)/libfawkesnaoqibroker.so interfaces

ifeq ($(OBJSSUBMAKE),1)
all: $(WARN_TARGETS)

.PHONY: warning_naoqi info_naoqiroot
warning_naoqi:
	$(SILENT)echo -e "$(INDENT_PRINT)--> $(TRED)Omitting Nao Plugins$(TNORMAL) (NaoQi not found)"

info_naoqiroot:
	$(SILENT)echo -e "$(INDENT_PRINT)--> $(TGREEN)NaoQi root:$(TNORMAL) $(NAOQIROOT)"
endif

include $(BUILDSYSDIR)/base.mk

