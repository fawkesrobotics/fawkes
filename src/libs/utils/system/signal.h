
/***************************************************************************
 *  signal.h - This header defines a true OOo signal handler
 *  based on
 *  Douglas C. Schmidt
 *  "Applying Design Patterns to Simplify Signal Handling"
 *  http://www.cs.wustl.edu/~schmidt/signal-patterns.html
 *
 *  Generated: Thu Jan 12 22:44:59 2006 (from FireVision)
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

#ifndef __UTILS_SYSTEM_SIGNAL_H_
#define __UTILS_SYSTEM_SIGNAL_H_

#include <signal.h>

namespace fawkes {

class SignalHandler {
 public:
  virtual ~SignalHandler() {}
  virtual void handle_signal(int signal)   = 0;
};


class SignalManager {

 public:
  static SignalManager *  instance();
  static void             finalize();
  static SignalHandler *  register_handler(int signum, SignalHandler *handler);
  static void             unregister_handler(int signum);
  static void             unregister_handler(SignalHandler *handler);
  static void             ignore(int signum);

 private:
  // Guard constructors, make sure we are a singleton
  SignalManager();
  SignalManager(const SignalManager& cc);

  static SignalManager *__instance;

  // Entry point adapter installed into <sigaction> 
  // (must be a static method or a stand-alone 
  // extern "C" function).
  static void dispatcher (int signum);

  // restores default signal handler, called by unregister_*
  static void restore_default(int signum);

  // Table of pointers to concrete <SignalHandler>s
  // registered by applications.  NSIG is the number of 
  // signals defined in <signal.h>.
  static SignalHandler *  __signal_handlers[NSIG];

};

} // end namespace fawkes

#endif
