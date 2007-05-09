
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

OBJS_qa_utils_logger = qa_logger.o
LIBS_qa_utils_logger = utils

OBJS_qa_utils_liblogger = qa_liblogger.o
LIBS_qa_utils_liblogger = utils

OBJS_all =	$(OBJS_test_plugin)			\
		$(OBJS_qa_utils_plugin)			\
		$(LIBS_qa_utils_ipc_shmem)		\
		$(LIBS_qa_utils_ipc_shmem_lock)		\
		$(LIBS_qa_utils_ipc_shmem_lowlevel)	\
		$(LIBS_qa_utils_ipc_msg)		\
		$(LIBS_qa_utils_ipc_semset)		\
		$(LIBS_qa_utils_hostinfo)		\
		$(LIBS_qa_utils_logger)			\
		$(LIBS_qa_utils_liblogger)

BINS_all =	$(BINDIR)/qa_utils_plugin		\
		$(BINDIR)/qa_utils_ipc_shmem		\
		$(BINDIR)/qa_utils_ipc_shmem_lock	\
		$(BINDIR)/qa_utils_ipc_shmem_lowlevel	\
		$(BINDIR)/qa_utils_ipc_msg		\
		$(BINDIR)/qa_utils_ipc_semset		\
		$(BINDIR)/qa_utils_hostinfo		\
		$(BINDIR)/qa_utils_logger		\
		$(BINDIR)/qa_utils_liblogger

PLUGINS_all =	$(PLUGINDIR)/test_plugin.so

include $(BASEDIR)/etc/buildsys/base.mk

