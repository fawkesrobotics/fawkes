
/***************************************************************************
 *  thread.h - base class for threads, implementation based on pthreads
 *
 *  Created: Thu Sep 14 13:06:18 2006
 *  Copyright  2006-2008  Tim Niemueller [www.niemueller.de]
 *
 *  $Id$
 *
 ****************************************************************************/

/*  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version. A runtime exception applies to
 *  this software (see LICENSE file mentioned below for details).
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU Library General Public License for more details.
 *
 *  You can read the full text in the LICENSE file in the doc directory. 
 */

#ifndef __CORE_THREADING_THREAD_H_
#define __CORE_THREADING_THREAD_H_

#include <sys/types.h>

#define forever while (1)

class WaitCondition;
class Mutex;
class Barrier;
class ReadWriteLock;
class ThreadNotificationListener;
class ThreadList;
template <typename Type> class LockList;

class Thread {
 public:
  friend class ThreadList;

  /** Thread operation mode.
   * A thread can operate in two different modes. In continuous mode the
   * thread is on it's own running continuously. No timing is done. You must
   * use this mode if you implement run() instead of loop(). In
   * wait-for-wakeup mode the thread will pause after each loop and wait for
   * an explicit wakeup. This is only guaranteed if you override loop() and
   * leave run() as it is. Have this in mind of chaos and havoc will
   * get you.
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

  virtual ~Thread();

  virtual void init();
          bool prepare_finalize();
  virtual bool prepare_finalize_user();
  virtual void finalize();
          void cancel_finalize();

  void start();
  void cancel();
  void join();
  void detach();
  void kill(int sig);

  bool operator==(const Thread &thread);

  void wakeup();
  void wakeup(Barrier *barrier);

  OpMode        opmode() const;
  const char *  name() const;
  pthread_t     thread_id() const;
  bool          started() const;
  bool          cancelled() const;
  bool          detached() const;

  static Thread *  current_thread();
  static Thread *  current_thread_noexc() throw();
  static pthread_t current_thread_id();

  static void      init_main();
  static void      destroy_main();

  static void      set_cancel_state(CancelState new_state, CancelState *old_state = 0);

  void set_delete_on_exit(bool del);

  void add_notification_listener(ThreadNotificationListener *notification_listener);
  void remove_notification_listener(ThreadNotificationListener *notification_listener);

 protected:
  Thread(const char *name);
  Thread(const char *name, OpMode op_mode);
  void exit();
  void test_cancel();
  void yield();

  void set_opmode(OpMode op_mode);
  void set_prepfin_conc_loop(bool concurrent);

  virtual void once();
  virtual void loop();

  bool       finalize_prepared;
  Mutex     *loop_mutex;

 private:
  Thread(const Thread &t);
  Thread(const char *name, pthread_t id);
  Thread & operator=(const Thread &t);
  static void * entry(void * pthis);
  void run();
  void __constructor(const char *name, OpMode op_mode);
  void set_finalize_sync_lock(ReadWriteLock *lock);
  void notify_of_failed_init();
  void notify_of_startup();
  void lock_sleep_mutex();

  static void init_thread_key();
  static void set_tsd_thread_instance(Thread *t);

  pthread_t      __thread_id;

  Barrier       *__startup_barrier;
  Mutex         *__prepfin_mutex;
  Mutex         *__sleep_mutex;
  WaitCondition *__sleep_condition;
  Barrier       *__barrier;

  ReadWriteLock *__finalize_sync_lock;

  bool           __started;
  bool           __cancelled;
  bool           __detached;
  bool           __delete_on_exit;
  char          *__name;

  OpMode         __op_mode;
  bool           __prepfin_conc_loop;

  LockList<ThreadNotificationListener *>  *__notification_listeners;

  static pthread_key_t   THREAD_KEY;
  static pthread_key_t   MAIN_THREAD_KEY;
  static pthread_mutex_t __thread_key_mutex;
};

#endif
