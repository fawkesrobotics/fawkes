
BASEDIR = ../../../..
include $(BASEDIR)/etc/buildsys/config.mk

CFLAGS = -g

LIBS_test_plugin = fawkesutils
OBJS_test_plugin = test_plugin.o

LIBS_qa_utils_plugin = fawkesutils fawkesplugin
OBJS_qa_utils_plugin = plugin.o

OBJS_qa_utils_ipc_shmem = qa_ipc_shmem.o
LIBS_qa_utils_ipc_shmem = fawkesutils

OBJS_qa_utils_ipc_shmem_lock = qa_ipc_shmem_lock.o
LIBS_qa_utils_ipc_shmem_lock = fawkesutils

OBJS_qa_utils_ipc_shmem_lowlevel = qa_ipc_shmem_lowlevel.o
LIBS_qa_utils_ipc_shmem_lowlevel = fawkesutils

OBJS_qa_utils_ipc_msg = qa_ipc_msg.o
LIBS_qa_utils_ipc_msg = fawkesutils

OBJS_qa_utils_ipc_semset = qa_ipc_semset.o
LIBS_qa_utils_ipc_semset = fawkesutils

OBJS_qa_utils_hostinfo = qa_hostinfo.o
LIBS_qa_utils_hostinfo = fawkesutils

OBJS_qa_utils_logger = qa_logger.o
LIBS_qa_utils_logger = fawkesutils

OBJS_qa_utils_liblogger = qa_liblogger.o
LIBS_qa_utils_liblogger = fawkesutils

OBJS_qa_utils_time = qa_time.o
LIBS_qa_utils_time = fawkesutils

OBJS_qa_utils_timebug = qa_timebug.o
LIBS_qa_utils_timebug = fawkescore fawkesutils

OBJS_qa_utils_angle = qa_angle.o
LIBS_qa_utils_angle = fawkesutils

OBJS_qa_utils_pathparser = qa_pathparser.o
LIBS_qa_utils_pathparser = fawkesutils

OBJS_qa_utils_filetype = qa_filetype.o
LIBS_qa_utils_filetype = fawkesutils

OBJS_qa_utils_rcsoft_graph = qa_rcsoft_graph.o
LIBS_qa_utils_rcsoft_graph = fawkesutils

OBJS_all =	$(OBJS_test_plugin)			\
		$(OBJS_qa_utils_plugin)			\
		$(OBJS_qa_utils_ipc_shmem)		\
		$(OBJS_qa_utils_ipc_shmem_lock)		\
		$(OBJS_qa_utils_ipc_shmem_lowlevel)	\
		$(OBJS_qa_utils_ipc_msg)		\
		$(OBJS_qa_utils_ipc_semset)		\
		$(OBJS_qa_utils_hostinfo)		\
		$(OBJS_qa_utils_logger)			\
		$(OBJS_qa_utils_liblogger)		\
		$(OBJS_qa_utils_time)			\
		$(OBJS_qa_utils_timebug)		\
		$(OBJS_qa_utils_angle)			\
		$(OBJS_qa_utils_pathparser)		\
		$(OBJS_qa_utils_filetype)		\
		$(OBJS_qa_utils_rcsoft_graph)

BINS_all =	$(BINDIR)/qa_utils_plugin		\
		$(BINDIR)/qa_utils_ipc_shmem		\
		$(BINDIR)/qa_utils_ipc_shmem_lock	\
		$(BINDIR)/qa_utils_ipc_msg		\
		$(BINDIR)/qa_utils_ipc_semset		\
		$(BINDIR)/qa_utils_hostinfo		\
		$(BINDIR)/qa_utils_logger		\
		$(BINDIR)/qa_utils_liblogger		\
		$(BINDIR)/qa_utils_time			\
		$(BINDIR)/qa_utils_timebug		\
		$(BINDIR)/qa_utils_pathparser		\
		$(BINDIR)/qa_utils_angle		\
		$(BINDIR)/qa_utils_filetype		\
		$(BINDIR)/qa_utils_rcsoft_graph

ifeq ($(OS),Linux)
  BINS_all += $(BINDIR)/qa_utils_ipc_shmem_lowlevel
endif

PLUGINS_all =	$(PLUGINDIR)/test_plugin.so

include $(BUILDSYSDIR)/base.mk

