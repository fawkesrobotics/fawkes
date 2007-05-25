
/***************************************************************************
 *  thread.cpp - implementation of threads, based on pthreads
 *
 *  Generated: Thu Sep 14 13:26:39 2006
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

#include <core/threading/thread.h>
#include <core/threading/mutex.h>
#include <core/threading/barrier.h>
#include <core/threading/wait_condition.h>
#include <core/threading/read_write_lock.h>
#include <core/threading/thread_finalizer.h>
#include <core/exceptions/software.h>

#include <pthread.h>
#include <unistd.h>
#include <typeinfo>

/** @def forever
 * Shortcut for "while (1)".
 * @relates Thread
 */

/** @class Thread core/threading/thread.h
 * Thread class encapsulation of pthreads.
 * This is the base class for all threads in Fawkes. Derive this class for
 * your thread.
 *
 * There are two major ways to implement threads. The recommended way is to
 * implement loop(). The default run() implementation will call loop()
 * continuously. An implicit cancel point is set after each loop.
 *
 * The thread can operate in two modes if the loop() implementation method is
 * chosen. The loop can either run continuously without a brake, or it can wait
 * for an explicit wakeup after each loop. Waiting for an explicit wakeup is the
 * default since this is the common use case in Fawkes.
 *
 * If you need a more complex behaviour you may also override run() and
 * implement your own thread behavior.
 * That that without taking special care the advanced debug functionality
 * will not available for threads.
 *
 * Special care has been taken to allow for proper initialization and
 * finalization. The special behavior of this routines can only be guaranteed
 * if the threads are managed properly, which is the case if we speak of the
 * Fawkes thread manager. This applies for the following paragraphs.
 *
 * The thread provides an init() routine which may be implemented
 * and is called just before the thread is started. If you make use of aspects
 * this is the first time when you can make use of aspects. These aspects
 * are not initialized in the constructor. init() is called just after the
 * aspect initialization. This is also the last chance to stop the thread
 * from being executed if you detect an error. If init() throws any exception
 * then the thread is never started.
 *
 * The methods prepare_finalize(), finalize() and cancel_finalize() are meant
 * to be used for finalization. First prepare_finalize() is called to prepare
 * finalization. At this stage the thread can veto and prevent finalization
 * from happening. For this prepare_finalize_user() has to be implemented
 * with the proper check, and maybe special actions that are needed to
 * prepare finalization (which may or may not happen independent from the
 * result of just this thread, see method description). Afterwards finalize()
 * may be called (independent of the prepare_finalize() result, see method
 * description). If finalize() is not executed the thread is notified with
 * cancel_finalize().
 *
 * @ingroup Threading
 * @ingroup FCL
 * @see Aspects
 * @see loop()
 * @see run()
 * @see example_barrier.cpp
 * @see example_mutex_count.cpp
 * @see example_rwlock.cpp
 * @see example_waitcond_serialize.cpp
 *
 * @author Tim Niemueller
 */


/** Default Constructor.
 * This constructor is protected so that Thread cannot be instantiated. This
 * constructor initalizes a few internal variables.
 */
Thread::Thread()
{
  constructor(NULL, OPMODE_CONTINUOUS);
}

/** Constructor.
 * This constructor is protected so that Thread cannot be instantiated. This
 * constructor initalizes a few internal variables.
 * @param op_mode Operation mode, see Thread::OpMode
 */
Thread::Thread(OpMode op_mode)
{
  constructor(NULL, op_mode);
}


/** Constructor.
 * This constructor is protected so that Thread cannot be instantiated. This
 * constructor initalizes a few internal variables. Uses continuous
 * operation mode.
 * @param name thread name, used for debugging, see Thread::name()
 */
Thread::Thread(const char *name)
{
  constructor(name, OPMODE_CONTINUOUS);
}


/** Constructor.
 * This constructor is protected so that Thread cannot be instantiated. This
 * constructor initalizes a few internal variables.
 * @param name thread name, used for debugging, see Thread::name()
 * @param op_mode Operation mode, see Thread::OpMode
 */
Thread::Thread(const char *name, OpMode op_mode)
{
  constructor(name, op_mode);
}


/** Initialize.
 * Kind of the base constructor.
 * @param name name of thread
 * @param op_mode operation mode
 */
void
Thread::constructor(const char *name, OpMode op_mode)
{
  this->op_mode = op_mode;
  this->_name   = name;
  if ( op_mode == OPMODE_WAITFORWAKEUP ) {
    sleep_condition = new WaitCondition();
    sleep_mutex = new Mutex();
  } else {
    sleep_condition = NULL;
    sleep_mutex = NULL;
  }
  thread_id = 0;
  barrier = NULL;
  cancelled = false;

  threadlist_sync_lock = NULL;
  finalize_mutex = new Mutex();
  loop_mutex = new Mutex();
  finalize_prepared = false;
}


/** Virtual destructor. */
Thread::~Thread()
{
  delete sleep_condition;
  sleep_condition = NULL;
  delete sleep_mutex;
  sleep_mutex = NULL;
  delete loop_mutex;
  delete finalize_mutex;
}


/** Copy constructor is NOT supported.
 * Using this constructor will cause havoc and chaos. It's only here
 * as private constructor to hide it! Therefore if you ever use it
 * internally it will always throw an exception.
 * @param t thread to copy.
 * @exception Exception Always thrown
 */
Thread::Thread(const Thread &t)
{
  throw Exception("You may not use copy constructor of class Thread");
}


/** Assignment is not allowed.
 * You may not assign one thread to another.
 * @param t thread to assign
 */
Thread &
Thread::operator=(const Thread &t)
{
  throw Exception("You may not use assignment operator of class Thread");
}


/** Initialize the thread.
 * This method is meant to be used in conjunction with aspects. Some parts
 * of the initialization may only happen after some aspect of the thread has
 * been initialized. Implement the init method with these actions. It is
 * guaranteed to be called just after all aspects have been initialized
 * and only once in the lifetime of the thread.
 * Throw an exception if any problem occurs and the thread should not run.
 *
 * Just because your init() routine suceeds and everything looks fine for
 * this thread does not automatically imply that it will run. If it belongs
 * to a group of threads in a ThreadList and any of the other threads fail
 * to initialize then no thread from this group is run and thus this thread
 * will never run. So you should not do any initialization that cannot be
 * undone in the destructor! finalize() will not be called in that case.
 *
 * @see Aspects
 */
void
Thread::init()
{
}


/** Prepare finalization.
 * Check if finalization at this point is possible and if so execute the
 * steps necessary to prepare for finalization. You also have to make sure
 * that this state of being able to finalize does not change until either
 * finalize() or cancel_finalize() is called.
 *
 * This method may return false, which means that at this point the thread
 * cannot be stopped safely. This might be due to a critical internal
 * condition that may hurt hardware if turned of right now. In this case
 * a logger should be used to log the reason for the failure. The check is
 * implemented in prepare_finalize_user(), which the user has to implement
 * if he needs special treatment.
 *
 * Even if the finalization is said to be unsafe and false is returned, the
 * caller may still decide to finalize this thread, for example if all
 * threads are shut down on application exit. So you may not rely on the
 * fact that the thread is not stopped if you return false.
 *
 * You may not override this method.
 *
 * This is only called on a running thread.
 *
 * @return true if the thread can be stopped and destroyed safely, false if
 * it has to stay alive
 * @see finalize()
 * @see cancel_finalize()
 */
bool
Thread::prepare_finalize()
{
  if ( finalize_prepared ) {
    throw CannotFinalizeThreadException("prepare_finalize() has already been called");
  }
  if ( ! threadlist_sync_lock ) {
    // Make sure that prepare_finalize() is called in full before any
    // cancel_finalize(), possible problem if multiple threads are
    // queried by a thread list
    finalize_mutex->lock();
    // Make sure loop() is not running
    if (op_mode != OPMODE_CONTINUOUS )  loop_mutex->lock();
  }
  bool prepared = prepare_finalize_user();
  finalize_prepared = true;
  if ( ! threadlist_sync_lock ) {
    finalize_mutex->unlock();
  }
  return prepared;
}


/** Prepare finalization user implementation.
 * This method is called by prepare_finalize(). If there can ever be a
 * situation where it is not safe to turn of a thread at some point in
 * time then implement this method to determine these unsafe states.
 *
 * An example that comes to my mind is our Katana arm. If you turn it off
 * it looses all power and collapses back upon itself. This may damage the
 * arm if it is not in a safe position. In this situation this method would
 * return false to indicate this problem.
 *
 * It is up to the user to decide if this should be taken for an implied
 * signal to get in such a safe state, if this is possible at all.
 *
 * This feature should be used rarely as it can have tremendous implications
 * on the performance and experience of the whole software. In any case your
 * implementation should somehow inform the user of the problem that caused
 * the finalization to fail. If you are using aspect use the LoggerAspect and
 * log the reason.
 *
 * The default implementation always allows finalization.
 * @return true, if the thread can be finalized, false otherwise.
 */
bool
Thread::prepare_finalize_user()
{
  return true;
}


/** Finalize the thread.
 * This method is executed just before the thread is canceled and destroyed.
 * It is always preceeded by a call to prepare_finalize(). If this is not
 * the case this is a failure. The condition can be checked with
 * the boolean variable finalize_prepared.
 *
 * This method is meant to be used in conjunction with aspects and to cover
 * thread inter-dependencies. This routine MUST bring the thread into a safe
 * state such that it may be canceled and destroyed afterwards. If there is
 * any reason that this cannot happen make your prepare_finalize() report so.
 *
 * This method is called by the thread manager just before the thread is
 * being cancelled. Here you can do whatever steps are necessary just before
 * the thread is cancelled. Note that you thread is still running and might
 * be in the middle of a loop, so it is not a good place to give up on all
 * resources used. Mind segmentation faults that could happen. Protect the
 * area with a mutex that you lock at the beginning of your loop and free
 * in the end, and that you lock at the beginning of finalize and then never
 * unlock. Also not that the finalization may be canceled afterwards. The
 * next thing that happens is that either the thread is canceled and destroyed
 * or that the finalization is canceled and the thread has to run again.
 *
 * This is only called on a running thread.
 *
 * The default implementation does nothing besides throwing an exception if
 * prepare_finalize() has not been called.
 *
 * @exception Exception thrown if prepare_finalize() has not been called.
 * @see prepare_finalize()
 * @see cancel_finalize()
 */
void
Thread::finalize()
{
}


/** Cancel finalization.
 * This means that something has happened (for example another thread from
 * the same plugin) has indicated that it can not be finalized. In that case
 * also this thread has to continue to run and the finalization is canceled.
 * The thread is expected to run after the finalization has been canceled as
 * if the finalization was never tried.
 *
 * This is only called on a running thread.
 * @see prepare_finalize()
 * @see finalize()
 */
void
Thread::cancel_finalize()
{
  if ( ! threadlist_sync_lock ) {
    finalize_mutex->lock();
    finalize_prepared = false;
    if ( op_mode != OPMODE_CONTINUOUS )  loop_mutex->unlock();
    finalize_mutex->unlock();
  } else {
    finalize_prepared = false;
  }
}


/** Call this method to actuall start.
 * This method has to be called after the thread has been instantiated and
 * initialized to startup.
 * @return true, if the thread started successfully, false otherwise. error()
 * will return the error value in that case
 */
bool
Thread::start()
{
  int err;
  if ( (err = pthread_create(&thread_id, NULL, Thread::entry, this)) != 0) {
    // An error occured
    return false;
  }

  cancelled = false;
  return true;
}


/** Entry point for the thread.
 * This is an utility method that acts as an entry point to the thread.
 * It is called automatically when you start the thread and will call run()
 * @param pthis a pointer to the instance that triggered the run of this method
 */
/* static */  void *
Thread::entry(void *pthis)
{
  Thread *t = (Thread *)pthis;
  t->run();
  return NULL;
}


/** Set the sync read/write lock.
 * For proper finalization synchronization of a list of threads a common
 * read/write lock is needed that all the threads to synchronize share.
 * See class information for more details.
 * @param lock new read/write lock, if NULL then single-thread locking
 * using mutexes is used.
 */
void
Thread::set_threadlist_sync_lock(ReadWriteLock *lock)
{
  threadlist_sync_lock = lock;
}

/** Exit the thread.
 * You may call this from within your run() method to exit the thread.
 * @see run()
 */
void
Thread::exit()
{
  pthread_exit(NULL);
}


/** Join the thread.
 * This waites for the thread to exit.
 */
void
Thread::join()
{
  void *dont_care;
  pthread_join(thread_id, &dont_care);
}


/** Detach the thread.
 * Memory claimed by the thread will be automatically freed after the
 * thread exits. You can no longer join this thread.
 */
void
Thread::detach()
{
  pthread_detach(thread_id);
}


/** Cancel a thread.
 * Use this to cancel the thread.
 */
void
Thread::cancel()
{
  if ( ! cancelled ) {
    cancelled = true;
    pthread_cancel(thread_id);
  }
}


/** Get operation mode.
 * @return opmode of thread.
 */
Thread::OpMode
Thread::opmode() const
{
  return op_mode;
}


/** Get thread name.
 * @return name of thread
 */
const char *
Thread::name() const
{
  if ( _name != NULL ) {
    return _name;
  } else {
    return typeid(this).name();
  }
}


/** Get name of thread.
 * This name is mainly used for debugging purposes. Give it a descriptive
 * name. Is nothing is given the raw class name is used.
 * @return thread name
 */


/** Set cancellation point.
 * Tests if the thread has been canceled and if so exits the thread.
 */
void
Thread::test_cancel()
{
  pthread_testcancel();
}


/** Check if two threads are the same.
 * @param thread Thread to compare this thread to.
 * @return true, if the threads are equal, false otherwise.
 */
bool
Thread::operator==(const Thread &thread)
{
  return ( pthread_equal(thread_id, thread.thread_id) != 0 );
}


/** Code to execute in the thread.
 * Executes loop() in each cycle. This is the default implementation and if
 * you need a more specific behaviour you can override this run() method and
 * ignore loop().
 */
void
Thread::run()
{
  if ( op_mode == OPMODE_WAITFORWAKEUP ) {
    sleep_mutex->lock();
    sleep_condition->wait(sleep_mutex);
    sleep_mutex->unlock();
  }
  forever {
    if ( threadlist_sync_lock ) {
      threadlist_sync_lock->lockForRead();
    } else {
      loop_mutex->lock();
    }
    loop();
    if ( threadlist_sync_lock ) {
      threadlist_sync_lock->unlock();
    } else {
      loop_mutex->unlock();
    }
    test_cancel();
    if ( barrier ) {
      barrier->wait();
      barrier = NULL;
    }
    if ( op_mode == OPMODE_WAITFORWAKEUP ) {
      sleep_mutex->lock();
      sleep_condition->wait(sleep_mutex);
      sleep_mutex->unlock();
    }
    usleep(0);
  }
}


/** Wake up thread.
 * If the thread is being used in wait for wakeup mode this will wake up the
 * waiting thread.
 */
void
Thread::wakeup()
{
  if ( op_mode == OPMODE_WAITFORWAKEUP ) {
    sleep_condition->wakeAll();
  }
}


/** Wake up thread and wait for barrier afterwards.
 * If the thread is being used in wait for wakeup mode this will wake up the
 * waiting thread. Additionally after the loop is finished 
 * @param barrier barrier to wait for after loop
 */
void
Thread::wakeup(Barrier *barrier)
{
  if ( op_mode != OPMODE_WAITFORWAKEUP )  return;

  if ( barrier == NULL ) {
    throw NullPointerException(" Thread::wakeup(): barrier must not be NULL");
  }
  this->barrier = barrier;
  sleep_condition->wakeAll();
}


/** Code to execute in the thread.
 * Implement this method to hold the code you want to be executed continously.
 */
void
Thread::loop()
{
}
