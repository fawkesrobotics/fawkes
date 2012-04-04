
/***************************************************************************
 *  thread.cpp - implementation of threads, based on pthreads
 *
 *  Created: Thu Sep 14 13:26:39 2006
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

#include <core/threading/thread.h>
#include <core/threading/mutex.h>
#include <core/threading/mutex_locker.h>
#include <core/threading/barrier.h>
#include <core/threading/wait_condition.h>
#include <core/threading/read_write_lock.h>
#include <core/threading/thread_finalizer.h>
#include <core/threading/thread_notification_listener.h>
#include <core/exceptions/software.h>
#include <core/exceptions/system.h>
#include <core/utils/lock_list.h>

#include <pthread.h>
#include <climits>
#include <unistd.h>
#include <cstring>
#include <cstdlib>
#include <cerrno>
#include <csignal>
#include <cstdio>

namespace fawkes {

/** @def forever
 * Shortcut for "while (1)".
 * @relates Thread
 */

/** @class Thread <core/threading/thread.h>
 * Thread class encapsulation of pthreads.
 * This is the base class for all threads in Fawkes. Derive this class for
 * your thread. Note that you have to set a meaningful name, as this name
 * is necessary for easier debugging and it is used for internal messaging
 * via the BlackBoard. Make sure that your name is unique throughout the
 * software. Using the class name with an additional modifier if it is instantiated
 * multiple times is a good bet.
 *
 * The thread can operate in two modes. The loop can either run continuously
 * without a brake, or it can wait for an explicit wakeup after each loop.
 * Waiting for an explicit wakeup is the default since this is the common use
 * case in Fawkes and also it is less risky, the developer will easier see that
 * his thread does not do anything then fixing that the thread takes all CPU time.
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
 * cancel_finalize(). Before finalize() is called the thread is stopped.
 *
 * The intialization and finalization procedures may be executed deferred and
 * concurrent to the running thread itself. The thread is only started however
 * it init() finished successfully.
 *
 * The call to prepare_finalize() is mutual exclusive with a concurrently running
 * loop() by default. This means that if the loop() blocks waiting for some event
 * prepare_finalize() will hang until this event happens. This can be prevented
 * with set_prepfin_conc_loop() which allows to set that prepare_finalize() and
 * loop() may be executed concurrently.
 * 
 * After prepare_finalize() has been run the thread implementation will stop the
 * loop() from being executed. However, the thread will still run, for example it will
 * wait for wakeup. This way it can be ensured that other threads will continue
 * to run even this thread is currently not running. An exception is the
 * ThreadList. For this Thread provides special synchronization features by
 * which it is possible to stop a thread in the very same loop iteration. That
 * means that if you have two threads that are woken up at the same time and
 * maybe even synchronize among each other it is guaranteed that both threads
 * will finish the running loop and never enter the next loop.
 * Before finalize() is called the thread shall be stopped (cancelled and joined).
 *
 * Because the finalization is done deferred and concurrent put all lengthy
 * finalization routines in finalize() and avoid this in the destructor, since
 * a long running destructor will harm the overall performance while with the
 * surrounding framework a long-running finalize() is acceptable.
 *
 * Please read the Fawkes documentation about guarantees (FawkesGuarantees in
 * the wiki) for information about the given guarantees. Several of these
 * guarantees are met if Thread is used in conjunction with ThreadList and the
 * guarantees have been specifically designed for painless plugin development.
 *
 * @ingroup Threading
 * @ingroup FCL
 * @see Aspects
 * @see loop()
 * @see run()
 * @see ThreadList
 * @see example_barrier.cpp
 * @see example_mutex_count.cpp
 * @see example_rwlock.cpp
 * @see example_waitcond_serialize.cpp
 *
 * @author Tim Niemueller
 */

/** @var bool Thread::finalize_prepared
 * True if prepare_finalize() has been called and was not stopped with a
 * cancel_finalize(), false otherwise.
 * This can also be used in finalize() to detect whether prepare_finalize() was
 * run or not.
 */

/** @var Mutex *  Thread::loop_mutex
 * Mutex that is used to protect a call to loop().
 * This mutex is locked just before loop() is called and unlocked right after it
 * has finished. So you can use this lock in your derivate to make sure that a
 * method does not run while the loop runs.
 * For example assume that we have a method set_parameter(int x). This method may
 * only be called if loop() is not running or unpredictable results will occur.
 * To do this you could write the method as
 * @code
 * MyThread::set_parameter(int x)
 * {
 *   loopinterrupt_antistarve_mutex->lock();
 *   loop_mutex->lock();
 *   // do what you need to do...
 *   loop_mutex->unlock();
 *   loopinterrupt_antistarve_mutex->unlock();
 * }
 * @endcode
 * See documentation for loopinterrupt_antistarve_mutex why you need to use two
 * mutexes here.
 */

/** @var Mutex *  Thread::loopinterrupt_antistarve_mutex
 * Mutex to avoid starvation when trying to lock loop_mutex.
 * If you want to interrupt the main loop only locking loop_mutex is not enough,
 * as this might make your try to lock it starve if the loop is running too fast
 * (for example on a continuous thread). Because of this you always need to
 * lock both mutexes. The anti-starve mutex will only be visited shortly and thus
 * allows you to lock it easily. This will then block the thread from trying to
 * lock the loop_mutex. See loop_mutex for an example.
 */

/** @fn const char * Thread::name() const
 * Get name of thread.
 * This name is mainly used for debugging purposes. Give it a descriptive
 * name. Is nothing is given the raw class name is used.
 * @return thread name
 */


/** We need not initialize this one timely by ourselves thus we do not use Mutex */
pthread_mutex_t Thread::__thread_key_mutex = PTHREAD_MUTEX_INITIALIZER;


/** Key used to store a reference to the thread object as thread specific data. */
pthread_key_t Thread::THREAD_KEY = PTHREAD_KEYS_MAX;

#define MAIN_THREAD_NAME "__MainThread__"

/** Standard thread flag: "thread is bad" */
const unsigned int Thread::FLAG_BAD = 0x00000001;

/** Constructor.
 * This constructor is protected so that Thread cannot be instantiated. This
 * constructor initalizes a few internal variables. Uses continuous
 * operation mode.
 * @param name thread name, used for debugging, see Thread::name()
 */
Thread::Thread(const char *name)
{
  __constructor(name, OPMODE_CONTINUOUS);
}


/** Constructor.
 * This constructor is protected so that Thread cannot be instantiated. This
 * constructor initalizes a few internal variables.
 * @param name thread name, used for debugging, see Thread::name()
 * @param op_mode Operation mode, see Thread::OpMode
 */
Thread::Thread(const char *name, OpMode op_mode)
{
  __constructor(name, op_mode);
}


/** Constructor.
 * This constructor is protected so that Thread cannot be instantiated. This
 * constructor initalizes a few internal variables.
 * This is used to create a Thread wrapper instance for an existing thread.
 * Use internally only!
 * @param name thread name, used for debugging, see Thread::name()
 * @param id thread ID of running thread
 */
Thread::Thread(const char *name, pthread_t id)
{
  __constructor(name, OPMODE_CONTINUOUS);
  __thread_id = id;
}


/** Initialize.
 * Kind of the base constructor.
 * @param name name of thread
 * @param op_mode operation mode
 */
void
Thread::__constructor(const char *name, OpMode op_mode)
{
  init_thread_key();

  __prepfin_conc_loop = false;
  __coalesce_wakeups  = false;
  __op_mode = op_mode;
  __name   = strdup(name);
  __notification_listeners = new LockList<ThreadNotificationListener *>();

  if ( __op_mode == OPMODE_WAITFORWAKEUP ) {
    __sleep_mutex        = new Mutex();
    __sleep_condition    = new WaitCondition(__sleep_mutex);
    __waiting_for_wakeup = true;
  } else {
    __sleep_condition    = NULL;
    __sleep_mutex        = NULL;
    __waiting_for_wakeup = false;
  }

  __thread_id       = 0;
  __flags           = 0;
  __barrier         = NULL;
  __started         = false;
  __cancelled       = false;
  __delete_on_exit  = false;
  __prepfin_hold    = false;
  __pending_wakeups = 0;

  loop_mutex = new Mutex();
  finalize_prepared = false;

  __loop_done = true;
  __loop_done_mutex = new Mutex();
  __loop_done_waitcond = new WaitCondition(__loop_done_mutex);

  loopinterrupt_antistarve_mutex = new Mutex();
  __prepfin_hold_mutex       = new Mutex();
  __prepfin_hold_waitcond    = new WaitCondition(__prepfin_hold_mutex);
  __startup_barrier          = new Barrier(2);
}


/** Virtual destructor. */
Thread::~Thread()
{
  __loop_done_waitcond->wake_all();
  yield();

  delete __sleep_condition;
  delete __sleep_mutex;
  delete loop_mutex;
  free(__name);
  delete __notification_listeners;
  delete loopinterrupt_antistarve_mutex;
  delete __startup_barrier;
  delete __prepfin_hold_mutex;
  delete __prepfin_hold_waitcond;
  delete __loop_done_waitcond;
  delete __loop_done_mutex;
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
 * will never run. In that situation finalize() is called for this very
 * instance, prepare_finalize() however is not called.
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
 * It is guaranteed that this method is only called for a running thread.
 *
 * @return true if the thread can be stopped and destroyed safely, false if
 * it has to stay alive
 * @see finalize()
 * @see cancel_finalize()
 */
bool
Thread::prepare_finalize()
{
  if ( ! __started ) {
    throw CannotFinalizeThreadException("Thread has not been started");
  }
  if ( finalize_prepared ) {
    throw CannotFinalizeThreadException("prepare_finalize() has already been called");
  }
  __prepfin_hold_mutex->lock();
  while (__prepfin_hold) {
    __prepfin_hold_waitcond->wait();
  }
  if (! __prepfin_conc_loop) {
    loopinterrupt_antistarve_mutex->lock();
    loop_mutex->lock();
  }
  finalize_prepared = true;
  bool prepared = prepare_finalize_user();
  if (! __prepfin_conc_loop) {
    loop_mutex->unlock();
    loopinterrupt_antistarve_mutex->unlock();
  }
  __prepfin_hold_mutex->unlock();
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
 * any reason that this cannot happen make your prepare_finalize() reports so.
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
 * Finalize is called on a thread just before it is deleted. It is guaranteed
 * to be called on a fully initialized thread (if no exception is thrown in
 * init()) (this guarantee holds in the Fawkes framework).
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
 * This is only called on a running thread after prepare_finalization() has
 * been called.
 *
 * @see prepare_finalize()
 * @see finalize()
 */
void
Thread::cancel_finalize()
{
  if ( ! __started ) {
    throw CannotFinalizeThreadException("Cannot cancel finalize, thread has not been started");
  }
  loop_mutex->lock();
  finalize_prepared = false;
  loop_mutex->unlock();
}


/** Call this method to start the thread.
 * This method has to be called after the thread has been instantiated and
 * initialized to start it. To meet the Fawkes guarantees you this may only
 * be called if the initialization of the thread has been successful.
 * @param wait if true this method will block until the thread is really
 * started, otherwise it will only initiate the startup and return immediately
 */
void
Thread::start(bool wait)
{
  int err;
  if (__started) {
    throw Exception("You cannot start the same thread twice!");
  }

  __cancelled = false;
  __detached  = false;
  __started   = true;
  __wait      = wait;

  if ( (err = pthread_create(&__thread_id, NULL, Thread::entry, this)) != 0) {
    // An error occured
    throw Exception("Could not start thread", err);
  }

  if (__wait)  __startup_barrier->wait();
}


void
Thread::lock_sleep_mutex()
{
  if (__sleep_mutex) {
    __sleep_mutex->lock();
  }
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

  // Can be used for easier debugging in gdb, need to make this accessible
  // printf("Thread %s (%lu) started\n", t->name(), t->thread_id());

  // Set thread instance as TSD
  set_tsd_thread_instance(t);

  // lock sleep mutex, needed such that thread waits for initial wakeup
  t->lock_sleep_mutex();

  // Notify listeners that this thread started
  t->notify_of_startup();

  // Thread is started now, thread that called start() will continue
  if (t->__wait)  t->__startup_barrier->wait();

  // Run thread
  t->loop_mutex->lock();
  t->once();
  t->loop_mutex->unlock();
  t->run();

  if ( t->__detached ) {
    // mark as stopped if detached since the thread will be deleted
    // after entry() is done
    t->__started = false;
  }

  // have no useful exit value
  return NULL;
}


/** Exit the thread.
 * You may call this from within your run() method to exit the thread.
 * @see run()
 */
void
Thread::exit()
{
  if ( __delete_on_exit ) {
    delete this;
  }

  __cancelled   = true;
  pthread_exit(NULL);
}


/** Join the thread.
 * This waites for the thread to exit.
 */
void
Thread::join()
{
  if ( __started ) {
    void *dont_care;
    pthread_join(__thread_id, &dont_care);
    __started = false;

    if ( __sleep_mutex != NULL ) {
      // We HAVE to release this sleep mutex under any circumstances, so we try
      // to lock it (locking a locked mutex or unlocking and unlocked mutex are undefined)
      // and then unlock it. This is for example necessary if a thread is cancelled, and
      // then set_opmode() is called, this would lead to a deadlock if the thread was
      // cancelled while waiting for the sleep lock (which is very likely)
      __sleep_mutex->try_lock();
      __sleep_mutex->unlock();
    }

    // Force unlock of these mutexes, otherwise the same bad things as for the sleep
    // mutex above could happen!
    loop_mutex->try_lock();
    loop_mutex->unlock();
  }
}


/** Detach the thread.
 * Memory claimed by the thread will be automatically freed after the
 * thread exits. You can no longer join this thread.
 */
void
Thread::detach()
{
  __detached = true;
  pthread_detach(__thread_id);
}


/** Cancel a thread.
 * Use this to cancel the thread.
 */
void
Thread::cancel()
{
  if ( __started && ! __cancelled ) {
    if ( pthread_cancel(__thread_id) == 0 ) {
      __cancelled = true;
    }
  }
}


/** Send signal to a thread.
 * Not that sending an unhandled signal might kill the whole process, not just the
 * thread!
 * @param sig signal to send.
 */
void
Thread::kill(int sig)
{
  pthread_kill(__thread_id, sig);
}


/** Get operation mode.
 * @return opmode of thread.
 */
Thread::OpMode
Thread::opmode() const
{
  return __op_mode;
}


/** Set operation mode.
 * This can be done at any time and the thread will from the next cycle on
 * run in the new mode.
 * @param op_mode new operation mode
 */
void
Thread::set_opmode(OpMode op_mode)
{
  if ( __started ) {
    throw Exception("Cannot set thread opmode while running");
  }

  if ( (__op_mode == OPMODE_WAITFORWAKEUP) &&
       (op_mode == OPMODE_CONTINUOUS) ) {
    __op_mode = OPMODE_CONTINUOUS;
    delete __sleep_condition;
    delete __sleep_mutex;
    __sleep_condition = NULL;
    __sleep_mutex = NULL;
  } else if ( (__op_mode == OPMODE_CONTINUOUS) &&
	      (op_mode == OPMODE_WAITFORWAKEUP) ) {
    __sleep_mutex     = new Mutex();
    __sleep_condition = new WaitCondition(__sleep_mutex);
    __op_mode = OPMODE_WAITFORWAKEUP;
  }
}


/** Set concurrent execution of prepare_finalize() and loop().
 * Usually calls to prepare_finalize() and a running loop() are mutually exclusive.
 * The prepare_finalize() call will wait for the current loop() run to finish before
 * calling the user implementation. If you have a thread that blocks in its loop for
 * example in a blocking system call this would lead to a dead-lock if no condition
 * that makes the loop finish occurs. For this reason this method has been added.
 * If you set this to true then prepare_finalize() can be executed concurrent to
 * a running loop() call. If this is critical for parts of loop() you have to
 * protect the critical sections by yourself. Use this sparsely and be sure you really
 * know what you are doing. This method is necessary in some situations and powerful
 * if used wisely. By default a thread will enforce mutual exclusion.
 * @param concurrent true to allow concurrent execution of prepare_finalize() and loop(),
 * false to enforce mutual exclusion (the latter being the default)
 */
void
Thread::set_prepfin_conc_loop(bool concurrent)
{
  __prepfin_conc_loop = concurrent;
}


/** Set wakeup coalescing.
 * The standard behavior of multiple calls to wakeup() (before the thread actually
 * got woken up, for instance because a loop iteration was still running) is to
 * execute one iteration for each wakeup. When setting coalescing, multiple calls
 * will only cause a single execution of the loop.
 * @param coalesce true to coalesce wakeups, false to keep the original behavior
 */
void
Thread::set_coalesce_wakeups(bool coalesce)
{
  if ( __op_mode == OPMODE_CONTINUOUS ) {
    // nothing is using the value, just write it
    __coalesce_wakeups = coalesce;
  } else {
    // protect usage for calls to wakeup()
    MutexLocker lock(__sleep_mutex);
    __coalesce_wakeups = coalesce;
  }
}


/** Set name of thread.
 * If you want a more descriptive thread name you can do so by calling this method
 * in your thread's constructor, and only in the constructor.
 * Use parameters similar to printf().
 * @param format format string
 */
void
Thread::set_name(const char *format, ...)
{
  va_list va;
  va_start(va, format);
  char *old_name = __name;
  if (vasprintf(&__name, format, va) == -1) {
    __name = old_name;
    throw OutOfMemoryException("Could not set new thread name for '%s'", __name);
  } else {
    free(old_name);
  }
  va_end(va);
}


/** Hold prepare_finalize().
 * In some situations you have to hold the finalization of a thread up to a certain
 * safe point. With set_prepfin_hold() you can do this. If you set \p hold to true
 * then a call to \c prepare_finalize() will block until \c set_prepfin_hold(false)
 * is called.
 * @param hold true to hold next call to \c prepare_finalize(), false to release it
 * @exception Exception thrown if \c prepare_finalize() has already been called before
 * trying to set \p hold to true.
 */
void
Thread::set_prepfin_hold(bool hold)
{
  __prepfin_hold_mutex->lock();
  if ( hold && finalize_prepared ) {
    __prepfin_hold_mutex->unlock();
    throw Exception("Thread(%s)::set_prepfin_hold: prepare_finalize() has "
		    "been called already()", __name);
  }
  __prepfin_hold = hold;
  if ( ! hold ) {
    __prepfin_hold_waitcond->wake_all();
  }
  __prepfin_hold_mutex->unlock();
}


/** Get ID of thread.
 * @return thread ID
 */
pthread_t
Thread::thread_id() const
{
  return __thread_id;
}


/** Check if thread has been started.
 * @return true if thread has been started, false otherwise
 */
bool
Thread::started() const
{
  return __started;
}


/** Check if thread has been cancelled.
 * @return true if the thread has been cancelled, false otherwise
 */
bool
Thread::cancelled() const
{
  return __cancelled;
}


/** Check if thread has been detached.
 * @return true if the thread has been detached, false otherwise
 */
bool
Thread::detached() const
{
  return __detached;
}


/** Check if the thread is running.
 * A thread is running if it currently is busy in its loop() or once() method.
 * @return true if the thread is running, false otherwise
 */
bool
Thread::running() const
{
  // loop_mutex is mutable and thus we can call the lock methods here
  if (loop_mutex->try_lock()) {
    loop_mutex->unlock();
    return false;
  } else {
    return true;
  }
}


/** Check if thread is currently waiting for wakeup.
 * A continuous thread is never waiting for wakeup and thus will always return
 * false. A wait-for-wakeup thread is waiting when it has passed the wakeup
 * barrier (if supplied) and is now waiting for the next call to wakeup()
 * to run again.
 * @return true if the thread is waiting, false otherwise
 */
bool
Thread::waiting() const
{
  if (__op_mode != OPMODE_WAITFORWAKEUP) {
    return false;
  } else {
    return __waiting_for_wakeup;
  }
}

/** Set cancellation point.
 * Tests if the thread has been canceled and if so exits the thread.
 */
void
Thread::test_cancel()
{
  pthread_testcancel();
}


/** Yield the processor to another thread or process.
 * This will suspend the execution of the current thread in favor of other
 * threads. The thread will then be re-scheduled for later execution.
 * Use this method to make sure that other threads get a chance to get the CPU
 * for example if your thread is waiting for results from other threads.
 */
void
Thread::yield()
{
#ifdef __USE_GNU
  pthread_yield();
#else
  usleep(0);
#endif
}


/** Check if two threads are the same.
 * @param thread Thread to compare this thread to.
 * @return true, if the threads are equal, false otherwise.
 */
bool
Thread::operator==(const Thread &thread)
{
  return ( pthread_equal(__thread_id, thread.__thread_id) != 0 );
}


/** Code to execute in the thread.
 * Executes loop() in each cycle. This is the default implementation and if
 * you need a more specific behaviour you can override this run() method and
 * ignore loop().
 * Although this method is declared virtual, it should not be overridden, other
 * than with the following trivial snippet:
 * @code
 * protected: virtual void run() { Thread::run(); }
 * @endcode
 * The reason not to do other changes is that it contains complex house keeping
 * code that the system relies on. The reason for still allowing the override is
 * solely to make reading back traces in your debugger easier. Because now there
 * the class name of the thread sub-class will appear in the back trace, while
 * it would not otherwise.
 */
void
Thread::run()
{
  if ( __op_mode == OPMODE_WAITFORWAKEUP ) {
    // Wait for initial wakeup
    // __sleep_mutex has been locked in entry() already!
    while (__pending_wakeups == 0) {
      __waiting_for_wakeup = true;
      __sleep_condition->wait();
    }
    __pending_wakeups -= 1;
    __sleep_mutex->unlock();
  }

  forever {

    loopinterrupt_antistarve_mutex->stopby();

    loop_mutex->lock();
    if ( ! finalize_prepared ) {
      __loop_done = false;
      loop();
    }
    loop_mutex->unlock();

    __loop_done_mutex->lock();
    __loop_done = true;
    __loop_done_mutex->unlock();
    __loop_done_waitcond->wake_all();

    test_cancel();
    if ( __op_mode == OPMODE_WAITFORWAKEUP ) {
      if ( __barrier ) {
	__sleep_mutex->lock();
        Barrier *b = __barrier;
        __barrier = NULL;
	__sleep_mutex->unlock();

	b->wait();

	__sleep_mutex->lock();
      } else {
	__sleep_mutex->lock();
      }

      while (__pending_wakeups == 0) {
	__waiting_for_wakeup = true;
	__sleep_condition->wait();
      }
      __pending_wakeups -= 1;
      __sleep_mutex->unlock();
    }
    yield();
  }
}


/** Wake up thread.
 * If the thread is being used in wait for wakeup mode this will wake up the
 * waiting thread.
 */
void
Thread::wakeup()
{
  if ( __op_mode == OPMODE_WAITFORWAKEUP ) {
    MutexLocker lock(__sleep_mutex);

    if ( __barrier ) {
      throw Exception("Thread(%s): wakeup() cannot be called if loop is running "
		      "with barrier already", __name);
    }

    if (__coalesce_wakeups) __pending_wakeups  = 1;
    else                    __pending_wakeups += 1;
    if (__waiting_for_wakeup) {
      // currently waiting
      __waiting_for_wakeup = false;
      __sleep_condition->wake_all();
    }
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
  if ( __op_mode != OPMODE_WAITFORWAKEUP )  return;

  if ( barrier == NULL ) {
    throw NullPointerException("Thread(%s)::wakeup(): barrier must not be NULL", __name);
  }

  MutexLocker lock(__sleep_mutex);
  if ( ! __waiting_for_wakeup && __barrier) {
    throw Exception("Thread %s already running with barrier, cannot wakeup %i  %p", __name,
                    __waiting_for_wakeup, __barrier);
  }

  __pending_wakeups += 1;
  __barrier = barrier;
  if (__waiting_for_wakeup) {
    // currently waiting
    __waiting_for_wakeup = false;
    __sleep_condition->wake_all();
  }
}


/** Wait for the current loop iteration to finish. */
void
Thread::wait_loop_done()
{
  __loop_done_mutex->lock();
  while (! __loop_done) {
    __loop_done_waitcond->wait();
  }
  __loop_done_mutex->unlock();
}

/** Code to execute in the thread.
 * Implement this method to hold the code you want to be executed continously.
 * If you do not implement this method, the default is that the thread will exit.
 * This is useful if you choose to only implement once().
 */
void
Thread::loop()
{
  if ( __delete_on_exit ) {
    delete this;
  }
  pthread_exit(NULL);
}


/** Execute an action exactly once.
 * This code is executed once and only once right after the thread is started
 * before loop() is called.
 * This is useful if you want to implement an one-shot background job. Just implement
 * once() and leave once() untouched. Start the thread and detach it and it will just
 * do its job and then die automatically. If you use set_delete_on_exit(true) even the
 * Thread instance will be automatically deleted.
 */
void
Thread::once()
{
}


/** Set whether the thread should be deleted on exit.
 * If you set this to true the thread instance is deleted if the threads exits
 * (only on internal exits, not if you cancel the thread!).
 * This is particularly useful if you only implement once() and not loop().
 * @param del true to delete thread on exit, false otherwise
 */
void
Thread::set_delete_on_exit(bool del)
{
  __delete_on_exit = del;
}


/** Check if wakeups are pending.
 * @return true if at least one more loop iteration has been queued (wakeup() has
 * been called), false otherwise
 */
bool
Thread::wakeup_pending()
{
  MutexLocker lock(__sleep_mutex);
  return (__pending_wakeups > 0);
}

/** Set flag for the thread.
 * The first two bytes of the flags are reserved for custom usage from the outside
 * and they are never used internally. The last two bytes are used to indicate
 * internal states, like flagging a thread as bad (timing was not ok). Setting
 * the latter bits may have influence on the inner workings on the thread and
 * thus should only be done if you really know what you are doing.
 * @param flag flag to set
 * @see set_flags()
 */
void
Thread::set_flag(uint32_t flag)
{
  __flags |= flag;
}


/** Unset flag.
 * Unsets a specified flag.
 * @param flag flag to unset
 * @see set_flag()
 */
void
Thread::unset_flag(uint32_t flag)
{
  __flags &= 0xFFFFFFFF ^ flag;
}


/** Set all flags in one go.
 * @param flags flags
 */
void
Thread::set_flags(uint32_t flags)
{
  __flags = flags;
}


/** Check if FLAG_BAD was set.
 * This is a convenience method to check if FLAG_BAD has been set.
 * @return true if flag is set, false otherwise
 */
bool
Thread::flagged_bad() const
{
  return __flags & FLAG_BAD;
}


/** Add notification listener.
 * Add a notification listener for this thread.
 * @param notification_listener notification listener to add
 */
void
Thread::add_notification_listener(ThreadNotificationListener *notification_listener)
{
  __notification_listeners->push_back_locked(notification_listener);
}


/** Remove notification listener.
 * @param notification_listener notification listener to remove
 */
void
Thread::remove_notification_listener(ThreadNotificationListener *notification_listener)
{
  __notification_listeners->remove_locked(notification_listener);
}


/** Notify of successful startup.
 * This method is called internally in entry().
 */
void
Thread::notify_of_startup()
{
  __notification_listeners->lock();
  LockList<ThreadNotificationListener *>::iterator i = __notification_listeners->begin();
  while (i != __notification_listeners->end()) {
    if (! (*i)->thread_started(this)) {
      i = __notification_listeners->erase(i);
    } else {
      ++i;
    }
  }
  __notification_listeners->unlock();  
}


/** Notify of failed init.
 * This method is called by ThreadList.
 */
void
Thread::notify_of_failed_init()
{
  __notification_listeners->lock();
  LockList<ThreadNotificationListener *>::iterator i = __notification_listeners->begin();
  while (i != __notification_listeners->end()) {
    if ( ! (*i)->thread_init_failed(this) ) {
      i = __notification_listeners->erase(i);
    } else {
      ++i;
    }
  }
  __notification_listeners->unlock();
}


/** Intialize thread key.
 * For internal usage only.
 */
void
Thread::init_thread_key()
{
  pthread_mutex_lock(&__thread_key_mutex);
  if ( THREAD_KEY == PTHREAD_KEYS_MAX ) {
    // Has not been initialized, do it!
    int err;
    if ( (err = pthread_key_create(&THREAD_KEY, NULL)) != 0 ) {
      if ( ENOMEM == err ) {
	throw OutOfMemoryException("Could not create key for thread "
				   "specific data (reference to thread)");
      } else {
	throw Exception("Thread key for reference to thread could not be created", err);
      }
    }
  }
  pthread_mutex_unlock(&__thread_key_mutex);
}


/** Set thread instance in thread-specific data (TSD).
 * Use thread-specific data to store a reference to the Thread instance in the
 * pthread struct. Used by current_thread().
 * @param t thread to set specific data on
 */
void
Thread::set_tsd_thread_instance(Thread *t)
{
  int err = 0;
  if ( (err = pthread_setspecific(THREAD_KEY, t)) != 0 ) {
    if ( ENOMEM == err ) {
      throw OutOfMemoryException("Could not set specific data (reference to thread)");
    } else {
      throw Exception("Could not set specific data (reference to thread), unknown reason");
    }
  }
}


/** Initialize Thread wrapper instance for main thread.
 * This will create an internal Thread instance such that it can be guaranteed that
 */
void
Thread::init_main()
{
  init_thread_key();
  Thread *t = new Thread(MAIN_THREAD_NAME, pthread_self());
  set_tsd_thread_instance(t);
}


/** Destroy main thread wrapper instance.
 * This destroys the thread wrapper created with init_main(). Note that
 * this has to be called from the very same thread that init_main() was called
 * from, which should be the main thread (somewhere from main() on).
 */
void
Thread::destroy_main()
{
  Thread *t = current_thread();
  if ( strcmp(t->name(), MAIN_THREAD_NAME) == 0 ) {
    delete t;
  } else {
    throw Exception("Main thread can only be destroyed in main thread");
  }
}


/** Get the ID of the currently running thread.
 * This will return the ID of the thread in which's context this method was
 * called.
 * @return ID of thread context
 */
pthread_t
Thread::current_thread_id()
{
  return pthread_self();
}


/** Get the Thread instance of the currently running thread.
 * This will return the Thread instance of the thread in which's context this method was
 * called.
 * Note that only if the main application ensures to call init_main() it can be guaranteed
 * that this value is not NULL.
 * @return Thread instance of the current thread
 * @exception Exception thrown if this method is called before either init_main() is
 * called or any one thread has been started.
 */
Thread *
Thread::current_thread()
{
  if ( THREAD_KEY == PTHREAD_KEYS_MAX ) {
    throw Exception("No thread has been initialized");
  }
  return (Thread *)pthread_getspecific(THREAD_KEY);
}


/** Similar to current_thread, but does never throw an exception.
 * This is a convenience method doing the same as current_thread(), but it never ever
 * throws an exception, rather it returns NULL in case of an error. This is necessary
 * if run from a C context.
 * @return Thread instance of the current thread
 */
Thread *
Thread::current_thread_noexc() throw()
{
  if ( THREAD_KEY == PTHREAD_KEYS_MAX ) {
    return 0;
  }
  return (Thread *)pthread_getspecific(THREAD_KEY);
}


/** Set the cancel state of the current thread.
 * The cancel state can only be set on the current thread. Please also
 * consider the documentation for pthread_setcancelstate().
 * @param new_state new cancel state
 * @param old_state old cancel state
 */
void
Thread::set_cancel_state(CancelState new_state, CancelState *old_state)
{
  int oldstate = PTHREAD_CANCEL_ENABLE;
  int newstate = PTHREAD_CANCEL_ENABLE;
  if ( new_state == CANCEL_DISABLED ) {
    newstate = PTHREAD_CANCEL_DISABLE;
  }

  pthread_setcancelstate(newstate, &oldstate);

  if ( old_state != NULL ) {
    if ( oldstate == PTHREAD_CANCEL_DISABLE ) {
      *old_state = CANCEL_DISABLED;
    } else {
      *old_state = CANCEL_ENABLED;
    }
  }
}


} // end namespace fawkes
