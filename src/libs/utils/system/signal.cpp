
/***************************************************************************
 *  signal.cpp - This header defines a trule OOo signal handler
 *  based on
 *  Douglas C. Schmidt
 *  "Applying Design Patterns to Simplify Signal Handling"
 *  http://www.cs.wustl.edu/~schmidt/signal-patterns.html
 *
 *  Generated: Thu Jan 12 22:55:34 2006
 *  Copyright  2005-2006  Tim Niemueller [www.niemueller.de]
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

#include <utils/system/signal.h>
#include <cstdlib>

namespace fawkes {

/** @class SignalHandler utils/system/signal.h
 * Interface for signal handling.
 * Derive this class and implement handle_signal() to handle signals.
 * The handler must then be registered via SignalManager::register_handler().
 * From then on handle_signal() is called if the desired signal has been received.
 *
 * @fn SignalHandler::~SignalHandler()
 * Virtual destructor.
 *
 * @fn void SignalHandler::handle_signal(int signum)
 * Signal hanlding method.
 * Implement this method with the action you want to perform on the registered
 * signals.
 * @param signum signal number of triggered signal
 *
 * @author Tim Niemueller
 */

/** @class SignalManager utils/system/signal.h
 * System signal manager.
 * This class dispatches signals received from the system to the appropriate
 * handlers or sets a signal to be ignored.
 * This class is never instantiated but rather you just register a handler.
 * After you are done with signal handling call finalize() to free the
 * use resources and de-register all signal handlers at once.
 *
 * @author Tim Niemueller
 */

SignalManager *  SignalManager::__instance = NULL;
SignalHandler *  SignalManager::__signal_handlers[NSIG];

/** Invalid constructor. */
SignalManager::SignalManager()
{
}


/** Invalid copy constructor. */
SignalManager::SignalManager(const SignalManager &cc)
{
}


/** Get the SignalManager instance
 * @return SignalManager instance
 */
SignalManager *
SignalManager::instance()
{
  if ( __instance == NULL ) {
    __instance = new SignalManager();
    for (unsigned int i = 0; i < NSIG; ++i) {
      __signal_handlers[i] = NULL;
    }
  }

  return __instance;
}


/** Finalize (and free) the SignalManager instance, this does NOT
 * implicitly delete the signal handlers, you have to do this by yourself
 */
void
SignalManager::finalize()
{
  if ( __instance != NULL ) {
    for (unsigned int i = 0; i < NSIG; ++i) {
      restore_default(i);
    }
    delete __instance;
    __instance = NULL;
  }
}


/** Register a SignalHandler for a signal
 * @param signum The signal number from <signal.h>
 * @param handler The SignalHandler that should handle this event
 * @return The SignalManager registered before, maybe NULL if there was none
 */
SignalHandler *
SignalManager::register_handler(int signum, SignalHandler *handler)
{
  if (signum < NSIG) {
    SignalHandler *old = __signal_handlers[signum];
    __signal_handlers[signum] = handler;

    // Register the <dispatcher> to handle this <signum>.
    struct sigaction sa;
    sa.sa_handler = SignalManager::dispatcher;
    sigemptyset (&sa.sa_mask);
    sa.sa_flags = 0;
    sigaction (signum, &sa, 0);

    return old;
  } else {
    return NULL;
  }
}


/** Unregister a SignalHandler for a signal
 * @param signum The signal number from <signal.h>
 */
void
SignalManager::unregister_handler(int signum)
{
  restore_default(signum);
}


/** Unregister a SignalHandler for a signal
 * @param handler The SignalHandler you want to unregister, will unregister
 *                all signals this handler was registered for
 */
void
SignalManager::unregister_handler(SignalHandler *handler)
{

  for (unsigned int i = 0; i < NSIG; ++i) {
    if ( __signal_handlers[i] == handler ) {
      restore_default(i);
    }
  }
}


void
SignalManager::restore_default(int signum)
{
  if (signum < NSIG) {
    __signal_handlers[signum] = NULL;

    // ignore this signal
    struct sigaction sa;
    sa.sa_handler = SIG_DFL;
    sigemptyset (&sa.sa_mask);
    sa.sa_flags = 0;
    sigaction (signum, &sa, 0);
  }
}


/** Ignore a signal
 * @param signum The signal number from <signal.h>
 */
void
SignalManager::ignore(int signum)
{
  if (signum < NSIG) {
    __signal_handlers[signum] = NULL;

    // ignore this signal
    struct sigaction sa;
    sa.sa_handler = SIG_IGN;
    sigemptyset (&sa.sa_mask);
    sa.sa_flags = 0;
    sigaction (signum, &sa, 0);
  }
}


/** Dispatch incoming signal to appropriate handler.
 * @param signum signal received.
 */
void
SignalManager::dispatcher(int signum)
{
  if (__signal_handlers[signum] != NULL) {
    __signal_handlers[signum]->handle_signal(signum);
  }
}

} // end namespace fawkes
