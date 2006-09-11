
BASEDIR = ../../..

LIBS_test_plugin = utils
OBJS_test_plugin = test_plugin.o

LIBS_qa_utils_plugin = utils
OBJS_qa_utils_plugin = plugin.o
CFLAGS_plugin = -DPLUGINDIR=\"$(CURDIR)/$(PLUGINDIR)\"

BINS_all = $(BINDIR)/qa_utils_plugin
PLUGINS_all = $(PLUGINDIR)/test_plugin.so

include $(BASEDIR)/etc/buildsys/base.mk

