
/***************************************************************************
 *  qa_logger.cpp - QA for Logger
 *
 *  Generated: Wed Jan 17 14:19:45 2007
 *  Copyright  2005-2007  Tim Niemueller [www.niemueller.de]
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

// Do not include in api reference
///@cond QA

#include <core/threading/thread.h>
#include <utils/system/signal.h>
#include <utils/logging/console.h>
#include <utils/logging/multi.h>
#include <utils/logging/logger.h>

#include <cstdio>

using namespace fawkes;

class LoggerQAThread : public Thread
{
 public:
  LoggerQAThread(const char *name, Logger *logger)
    : Thread(name)
  {
    this->logger = logger;
    i = 0;
  }

  virtual void loop()
  {
    ++i;
    printf("%s: Testing: %i\n", name(), i);
    logger->log_info(name(), "Testing: %i", i);
  }

 private:
  unsigned int i;
  Logger *logger;
};


class LoggerQAMain : public SignalHandler
{
 public:
  LoggerQAMain()
  {
    cl = ml = NULL;
    t1 = t2 = t3 = t4 = t5 = t6 = NULL;
  }

  ~LoggerQAMain()
  {
    delete t1;
    delete t2;
    delete t3;
    delete t4;
    delete t5;
    delete t6;
    // also deletes cl!
    delete ml;
  }

  virtual void handle_signal(int signum)
  {
    printf("Signal received, cancelling threads\n");
    t1->cancel();
    t2->cancel();
    t3->cancel();
    t4->cancel();
    t5->cancel();
    t6->cancel();
    printf("Threads cancelled\n");
  }

  void run()
  {
    cl = new ConsoleLogger();
    ml = new MultiLogger(cl);

    t1 = new LoggerQAThread("L-1-", ml);
    t2 = new LoggerQAThread("L-2-", ml);
    t3 = new LoggerQAThread("L-3-", ml);
    t4 = new LoggerQAThread("L-4-", ml);
    t5 = new LoggerQAThread("L-5-", ml);
    t6 = new LoggerQAThread("L-6-", ml);

    t1->start();
    t2->start();
    t3->start();
    t4->start();
    t5->start();
    t6->start();
    t1->join();
    t2->join();
    t3->join();
    t4->join();
    t5->join();
    t6->join();
  }
  
 private:
  Logger *cl;
  Logger *ml;
  LoggerQAThread *t1;
  LoggerQAThread *t2;
  LoggerQAThread *t3;
  LoggerQAThread *t4;
  LoggerQAThread *t5;
  LoggerQAThread *t6;
};

int
main(int argc, char **argv)
{

  /*
  ConsoleLogger cl;

  Exception e("Test Exception");

  cl.log_debug("QA", "DEBUG test output %i", 1);
  cl.log_info("QA", "DEBUG test output %i", 2);
  cl.log_warn("QA", "DEBUG test output %i", 3);
  cl.log_error("QA", "DEBUG test output %i", 4);

  cl.log_debug("QA", e);
  cl.log_info("QA", e);
  cl.log_warn("QA", e);
  cl.log_error("QA", e);

  ConsoleLogger *clp = new ConsoleLogger();

  clp->log_debug("QA", "DEBUG test output %i", 1);
  clp->log_info("QA", "DEBUG test output %i", 2);
  clp->log_warn("QA", "DEBUG test output %i", 3);
  clp->log_error("QA", "DEBUG test output %i", 4);

  clp->log_debug("QA", e);
  clp->log_info("QA", e);
  clp->log_warn("QA", e);
  clp->log_error("QA", e);

  LoggerTestThread *tt = new LoggerTestThread(clp);
  tt->start();
  tt->join();
  delete tt;

  delete clp;
  */

  LoggerQAMain main;
  SignalManager::register_handler(SIGINT, &main);
  main.run();
  SignalManager::finalize();

  return 0;
}

/// @endcond
