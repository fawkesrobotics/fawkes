
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
 *  along with this program; if not, write to the Free Software
 *  Foundation, Inc., 59 Temple Place - Suite 330, Boston, MA 02111-1307, USA.
 */

#ifndef __CORE_THREADING_THREAD_H_
#define __CORE_THREADING_THREAD_H_

/** Shortcut for "while (1)"
 */
#define forever while (1)

/** Thread class encapsulation of pthreads.
 * This is the base class for all threads in Fawkes. Derive this class for
 * your thread.
 *
 * There are two major ways to implement threads. The recommended way is to
 * implement loop(). The default run() implementation will call loop()
 * continuously. An implicit cancel point is set after each loop.
 *
 * If you need a more complex behaviour you may also override run() and
 * implement your own thread behavior.
 * That that without taking special care the advanced debug functionality
 * will not available for threads.
 *
 * @see loop()
 * @see run()
 * @author Tim Niemueller
 */
class Thread {
 public:

  /** Virtual empty destructor.
   */
  virtual ~Thread() {}

  /** Call this method to actuall start.
   * This method has to be called after the thread has been instantiated and
   * initialized to startup.
   * @return true, if the thread started successfully, false otherwise. error()
   * will return the error value in that case
   */
  bool start();

  /** Cancel a thread.
   * Use this to cancel the thread.
   * @return 
   */
  void cancel();

  /** Join the thread.
   */
  void join();

  /** Detach the thread.
   */
  void detach();

  /** Check if two threads are the same.
   * @param thread Thread to compare this thread to.
   * @return true, if the threads are equal, false otherwise.
   */
  bool operator==(const Thread &thread);

 protected:
  /** Exit the thread.
   * You may call this from within your run() method to exit the thread.
   * @see run()
   */
  void exit();

  /** Set cancellation point
   */
  void test_cancel();

  /** Code to execute in the thread.
   * Executes loop() in each cycle. This is the default implementation and if
   * you need a more specific behaviour you can override this run() method and
   * ignore loop().
   */
  virtual void run();

  /** Code to execute in the thread.
   * Implement this method to hold the code you want to be executed continously.
   */
  virtual void loop();

 private:
  /** Entry point for the thread.
   * This is an utility method that acts as an entry point to the thread.
   * It is called automatically when you start the thread and will call run()
   * @param pthis a pointer to the instance that triggered the run of this method
   */
  static void * entry(void * pthis);


 private:

  // Do not use pthread_t here to avoid including pthread.h
  /* pthread_t */ unsigned long int thread_id;

};



#endif
