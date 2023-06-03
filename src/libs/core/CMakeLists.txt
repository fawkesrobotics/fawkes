# *****************************************************************************
# CMake Build System for Fawkes
# -------------------
# Copyright (C) 2023 by Tarik Viehmann and Daniel Swoboda
#
# *****************************************************************************
#
# This program is free software: you can redistribute it and/or modify it under
# the terms of the GNU General Public License as published by the Free Software
# Foundation, either version 3 of the License, or (at your option) any later
# version.
#
# This program is distributed in the hope that it will be useful, but WITHOUT
# ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS
# FOR A PARTICULAR PURPOSE.  See the GNU General Public License for more
# details.
#
# You should have received a copy of the GNU General Public License along with
# this program.  If not, see <http://www.gnu.org/licenses/>.
#
# *****************************************************************************

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
add_subdirectory(tests)
target_link_libraries(fawkescore pthread stdc++ m rt)
target_include_directories(fawkescore PUBLIC core)
target_compile_options(fawkescore PUBLIC -DHAVE_EXECINFO)
