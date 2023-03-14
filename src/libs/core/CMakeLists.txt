add_library(
  fawkescore SHARED
  exception.cpp
  # qa/qa_mutex_sync.cpp
  # qa/qa_rwlock.cpp
  # qa/qa_barrier.cpp
  # qa/qa_waitcond_serialize.cpp
  # qa/qa_waitcond.cpp
  # qa/qa_mutex_count.cpp
  # qa/qa_exception.cpp
  plugin.cpp
  threading/scoped_rwlock.cpp
  threading/barrier.cpp
  threading/thread_finalizer.cpp
  threading/thread_loop_listener.cpp
  threading/mutex.cpp
  threading/thread_notification_listener.cpp
  threading/spinlock.cpp
  threading/interruptible_barrier.cpp
  threading/thread.cpp
  threading/thread_initializer.cpp
  threading/mutex_locker.cpp
  threading/thread_collector.cpp
  threading/thread_list.cpp
  threading/recursive_mutex.cpp
  threading/refc_rwlock.cpp
  threading/wait_condition.cpp
  threading/read_write_lock.cpp
  utils/refcount.cpp
  exceptions/software.cpp
  exceptions/system.cpp
  # tests/catch2_main.cpp
  # tests/test_wait_condition.cpp
  # tests/test_circular_buffer.cpp
)
target_link_libraries(fawkescore pthread stdc++ m rt)
target_include_directories(fawkescore PUBLIC core)
target_compile_options(fawkescore PUBLIC -DHAVE_EXECINFO)
