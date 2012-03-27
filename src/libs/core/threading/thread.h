
/***************************************************************************
 *  thread.h - base class for threads, implementation based on pthreads
 *
 *  Created: Thu Sep 14 13:06:18 2006
 *  Copyright  2006-2009  Tim Niemueller [www.niemueller.de]
 *
 ****************************************************************************/

/*  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version. A runtime exception applies to
 *  this software (see LICENSE.GPL_WRE file mentioned below for details).
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU Library General Public License for more details.
 *
 *  Read the full text in the LICENSE.GPL_WRE file in the doc directory.
 */

#ifndef __CORE_THREADING_THREAD_H_
#define __CORE_THREADING_THREAD_H_

#include <sys/types.h>
#include <stdint.h>

#define forever while (1)

namespace fawkes {


class WaitCondition;
class Mutex;
class Barrier;
class ThreadNotificationListener;
class ThreadList;
template <typename Type> class LockList;

class Thread {
 public:
  friend class ThreadList;

  /** Thread operation mode.
   * A thread can operate in two different modes. In continuous mode the
   * thread is on it's own running continuously. No timing is done. The loop() is
   * immediately called again after it has finished once. In wait-for-wakeup mode
   * the thread will pause after each loop and wait for an explicit wakeup.
   */
  typedef enum {
    OPMODE_CONTINUOUS,		/**< operate in continuous mode (default) */
    OPMODE_WAITFORWAKEUP	/**< operate in wait-for-wakeup mode */
  } OpMode;

  /** Cancel state.
   * The current cancel state of a thread.
   */
  typedef enum {
    CANCEL_ENABLED,	/**< cancellation is possible */
    CANCEL_DISABLED	/**< thread cannot be cancelled */
  } CancelState;

  static const unsigned int FLAG_BAD;

  virtual ~Thread();

  virtual void init();
          bool prepare_finalize();
  virtual bool prepare_finalize_user();
  virtual void finalize();
          void cancel_finalize();

  void start(bool wait=true);
  void cancel();
  void join();
  void detach();
  void kill(int sig);

  bool operator==(const Thread &thread);

  void wakeup();
  void wakeup(Barrier *barrier);

  void wait_loop_done();

  OpMode        opmode() const;
  pthread_t     thread_id() const;
  bool          started() const;
  bool          cancelled() const;
  bool          detached() const;
  bool          running() const;
  bool          waiting() const;
  const char *  name() const { return __name; }

  void  set_flags(uint32_t flags);
  void  set_flag(uint32_t flag);
  void  unset_flag(uint32_t flag);
  bool  flagged_bad() const;

  static Thread *  current_thread();
  static Thread *  current_thread_noexc() throw();
  static pthread_t current_thread_id();

  static void      init_main();
  static void      destroy_main();

  static void      set_cancel_state(CancelState new_state, CancelState *old_state = 0);

  void set_delete_on_exit(bool del);
  void set_prepfin_hold(bool hold);

  void add_notification_listener(ThreadNotificationListener *notification_listener);
  void remove_notification_listener(ThreadNotificationListener *notification_listener);

 protected:
  Thread(const char *name);
  Thread(const char *name, OpMode op_mode);
  void exit();
  void test_cancel();
  void yield();
  virtual void run();

  void set_opmode(OpMode op_mode);
  void set_prepfin_conc_loop(bool concurrent = true);
  void set_coalesce_wakeups(bool coalesce = true);

  void set_name(const char *format, ...);

  virtual void once();
  virtual void loop();

  bool         wakeup_pending();

  bool           finalize_prepared;
  mutable Mutex *loop_mutex;
  Mutex         *loopinterrupt_antistarve_mutex;

 private:
  Thread(const Thread &t);
  Thread(const char *name, pthread_t id);
  Thread & operator=(const Thread &t);
  static void * entry(void * pthis);
  void __constructor(const char *name, OpMode op_mode);
  void notify_of_failed_init();
  void notify_of_startup();
  void lock_sleep_mutex();

  static void init_thread_key();
  static void set_tsd_thread_instance(Thread *t);

  pthread_t      __thread_id;

  Barrier       *__startup_barrier;
  mutable Mutex *__sleep_mutex;
  WaitCondition *__sleep_condition;
  unsigned int   __pending_wakeups;
  Barrier       *__barrier;

  bool           __loop_done;
  Mutex         *__loop_done_mutex;
  WaitCondition *__loop_done_waitcond;

  bool           __prepfin_hold;
  Mutex         *__prepfin_hold_mutex;
  WaitCondition *__prepfin_hold_waitcond;

  bool           __started;
  bool           __cancelled;
  bool           __detached;
  bool           __waiting_for_wakeup;
  bool           __delete_on_exit;
  bool           __wait;
  char          *__name;

  OpMode         __op_mode;
  bool           __prepfin_conc_loop;
  bool           __coalesce_wakeups;

  uint32_t       __flags;

  LockList<ThreadNotificationListener *>  *__notification_listeners;

  static pthread_key_t   THREAD_KEY;
  static pthread_key_t   MAIN_THREAD_KEY;
  static pthread_mutex_t __thread_key_mutex;
};


} // end namespace fawkes

#endif
