
/***************************************************************************
 *  qa_liblogger.cpp - Fawkes QA for LibLogger
 *
 *  Created: Mon May 07 17:04:10 2007
 *  Copyright  2006-2007  Tim Niemueller [www.niemueller.de]
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

/// @cond QA

#include <core/threading/thread.h>
#include <utils/system/signal.h>
#include <utils/system/argparser.h>
#include <utils/logging/liblogger.h>
#include <utils/logging/console.h>
#include <utils/logging/file.h>
#include <core/exceptions/system.h>

#include <netdb.h>
#include <cstdio>
#include <cstring>
#include <cstdlib>
#include <iostream>

#include <list>

using namespace std;

class LibLoggerQAThread : public Thread
{
public:
  LibLoggerQAThread(unsigned int thread_num, unsigned int sleep_time_usec)
    : Thread("LibLoggerQAThread")
  {
    this->sleep_time_usec = sleep_time_usec;
    this->thread_num      = thread_num;
    i = 0;
  }

  ~LibLoggerQAThread()
  {
  }

  virtual void loop()
  {
    if ( (thread_num % 4) == 0 ) {
      LibLogger::log_debug("LibLoggerQA", "%u: %u (debug)", thread_num, ++i);
    } else if ( (thread_num % 3) == 0 ) {
      LibLogger::log_info("LibLoggerQA", "%u: %u (info)", thread_num, ++i);
    } else if ( (thread_num % 2) == 0 ) {
      LibLogger::log_warn("LibLoggerQA", "%u: %u (warn)", thread_num, ++i);
    } else {
      LibLogger::log_error("LibLoggerQA", "%u: %u (error)", thread_num, ++i);
    }
    usleep(sleep_time_usec);
  }

 private:
  unsigned int sleep_time_usec;
  unsigned int thread_num;
  unsigned int i;
};


class LibLoggerQAMain : public SignalHandler
{
 public:
  LibLoggerQAMain(ArgumentParser *argp)
  {
    unsigned int sleep_time_usec = 0;
    unsigned int num_threads     = 3;
    char *tmp;
    if ( (tmp = argp->getArgument("s")) != NULL ) {
      sleep_time_usec = atoi(tmp);
    }
    if ( (tmp = argp->getArgument("n")) != NULL ) {
      num_threads = atoi(tmp);
      if ( num_threads < 0 ) {
	num_threads = 3;
      }
    }

    threads.clear();
    for ( unsigned int i = 0; i < num_threads; ++i ) {
      threads.push_back( new LibLoggerQAThread(i, sleep_time_usec) );
    }
  }

  ~LibLoggerQAMain()
  {
    for ( tit = threads.begin(); tit != threads.end(); ++tit ) {
      delete (*tit);
    }
    threads.clear();
  }


  virtual void handle_signal(int signum)
  {
    printf("Signal received, cancelling threads\n");
    for ( tit = threads.begin(); tit != threads.end(); ++tit ) {
      (*tit)->cancel();
    }
    printf("Threads cancelled\n");
  }

  void run()
  {
    for ( tit = threads.begin(); tit != threads.end(); ++tit ) {
      (*tit)->start();
    }
    for ( tit = threads.begin(); tit != threads.end(); ++tit ) {
      (*tit)->join();
    }
  }

 private:
  list<Thread *> threads;
  list<Thread *>::iterator tit;
  ArgumentParser *argp;
};

int
main(int argc, char **argv)
{
  ArgumentParser *argp = new ArgumentParser(argc, argv, "s:n:");

  if ( argp->hasArgument("H") ) {
    cout << "Usage: " << argv[0] << "[-s n] [-n n]" << endl
	 << " -s n Sleep time for thres in usec" << endl
	 << " -H   this help message" << endl
	 << " -n n number of threads" << endl;
    return 0;
  }

  LibLoggerQAMain m(argp);
  SignalManager::register_handler(SIGINT, &m);
  SignalManager::ignore(SIGPIPE);

  LibLogger::init();
  LibLogger::add_logger(new FileLogger("qa_utils_liblogger.log"));
  LibLogger::add_logger(new ConsoleLogger());

  m.run();

  LibLogger::finalize();

  delete argp;
  return 0;
}

/// @endcond
