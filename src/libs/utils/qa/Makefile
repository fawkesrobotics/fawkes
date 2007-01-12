
BASEDIR = ../../../..

CFLAGS = -g

LIBS_test_plugin = utils
OBJS_test_plugin = test_plugin.o

LIBS_qa_utils_plugin = utils
OBJS_qa_utils_plugin = plugin.o

OBJS_qa_utils_ipc_shmem = qa_ipc_shmem.o
LIBS_qa_utils_ipc_shmem = utils

OBJS_qa_utils_ipc_shmem_lock = qa_ipc_shmem_lock.o
LIBS_qa_utils_ipc_shmem_lock = utils

OBJS_qa_utils_ipc_shmem_lowlevel = qa_ipc_shmem_lowlevel.o
LIBS_qa_utils_ipc_shmem_lowlevel = utils

OBJS_qa_utils_ipc_msg = qa_ipc_msg.o
LIBS_qa_utils_ipc_msg = utils

OBJS_qa_utils_ipc_semset = qa_ipc_semset.o
LIBS_qa_utils_ipc_semset = utils

OBJS_qa_utils_hostinfo = qa_hostinfo.o
LIBS_qa_utils_hostinfo = utils

BINS_all =	$(BINDIR)/qa_utils_plugin		\
		$(BINDIR)/qa_utils_ipc_shmem		\
		$(BINDIR)/qa_utils_ipc_shmem_lock	\
		$(BINDIR)/qa_utils_ipc_shmem_lowlevel	\
		$(BINDIR)/qa_utils_ipc_msg		\
		$(BINDIR)/qa_utils_ipc_semset		\
		$(BINDIR)/qa_utils_hostinfo

PLUGINS_all =	$(PLUGINDIR)/test_plugin.so

include $(BASEDIR)/etc/buildsys/base.mk

