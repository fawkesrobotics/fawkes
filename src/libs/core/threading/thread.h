
/***************************************************************************
 *  thread.h - base class for threads, implementation based on pthreads
 *
 *  Generated: Thu Sep 14 13:06:18 2006
 *  Copyright  2006  Tim Niemueller [www.niemueller.de]
 *
 *  $Id$
 *
 ****************************************************************************/

/*
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU Library General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program; if not, write to the Free Software Foundation,
 *  Inc., 51 Franklin Street, Fifth Floor, Boston, MA 02111-1307, USA.
 */

#ifndef __CORE_THREADING_THREAD_H_
#define __CORE_THREADING_THREAD_H_

#define forever while (1)

class WaitCondition;
class Mutex;
class Barrier;

class Thread {
 public:
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

  virtual ~Thread();

  virtual void init();

  bool start();
  void cancel();
  void join();
  void detach();

  bool operator==(const Thread &thread);

  void wakeup();
  void wakeup(Barrier *barrier);

  OpMode        opmode() const;
  const char *  name() const;

 protected:
  Thread();
  Thread(const char *name);
  Thread(OpMode op_mode);
  Thread(const char *name, OpMode op_mode);
  void exit();

  void test_cancel();

  virtual void run();
  virtual void loop();

 private:
  static void * entry(void * pthis);
  void constructor(const char *name, OpMode op_mode);

  // Do not use pthread_t here to avoid including pthread.h
  /* pthread_t */ unsigned long int thread_id;

  Mutex         *sleep_mutex;
  WaitCondition *sleep_condition;
  Barrier       *barrier;

  const char    *_name;

  bool           cancelled;

  OpMode         op_mode;

};



#endif
