
/***************************************************************************
 *  qa_avahi_resolver.cpp - QA for AvahiResolver
 *
 *  Created: Fri May 11 12:31:35 2007
 *  Copyright  2006-2007  Tim Niemueller [www.niemueller.de]
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

/// @cond QA

#include <netcomm/dns-sd/avahi_thread.h>
#include <netcomm/dns-sd/avahi_resolver_handler.h>

#include <core/exception.h>
#include <utils/system/signal.h>
#include <utils/system/argparser.h>

#include <cstdio>
#include <cstdlib>
#include <arpa/inet.h>
#include <netinet/in.h>

using namespace fawkes;

class QAAvahiResolverMain : public SignalHandler, public AvahiResolverHandler
{
 public:
  QAAvahiResolverMain(ArgumentParser *argp)
  {
    this->argp = argp;
    at = new AvahiThread();
    wait_for_name = false;
    wait_for_addr = false;
  }

  ~QAAvahiResolverMain()
  {
    delete at;
  }

  virtual void handle_signal(int signum)
  {
    at->cancel();
  }

  void run()
  {
    printf("Starting AvahiThread\n");
    at->start();

    printf("Waiting until AvahiThread has been initialized\n");
    at->wait_initialized();

    wait_for_name = argp->has_arg("n");
    wait_for_addr = argp->has_arg("a");

    const char *tmp;
    if ( (tmp = argp->arg("n")) != NULL ) {
      printf("Calling name resolver\n");
      at->resolve_name(tmp, this);
    }

    if ( (tmp = argp->arg("a")) != NULL ) {
      printf("Calling address resolver\n");
      struct sockaddr_in saddr;
      if ( inet_pton(AF_INET, tmp, &(saddr.sin_addr)) >= 0 ) {
	at->resolve_address((struct sockaddr *)&saddr, sizeof(saddr), this);
      }
    }

    printf("Waiting for thread\n");
    at->join();
  }
  
  virtual void resolved_name(char *name, struct sockaddr *addr, socklen_t addrlen)
  {
    char addrp[INET_ADDRSTRLEN];
    struct sockaddr_in *in_addr = (struct sockaddr_in *)addr;
    if ( inet_ntop(AF_INET, &(in_addr->sin_addr), addrp, sizeof(addrp)) ) {
      printf("'%s' resolved to '%s'\n", name, addrp);
    } else {
      printf("'%s' resolved, but could not transform address to presentation form\n", name);
    }

    free(name);
    free(addr);

    wait_for_name = false;
    if ( ! wait_for_name && ! wait_for_addr )  at->cancel();
  }


  virtual void resolved_address(struct sockaddr_in *addr, socklen_t addrlen, char *name)
  {
    char addrp[INET_ADDRSTRLEN];
    struct sockaddr_in *in_addr = (struct sockaddr_in *)addr;
    if ( inet_ntop(AF_INET, &(in_addr->sin_addr), addrp, sizeof(addrp)) ) {
      printf("Address %s resolved to %s\n", addrp, name);
    } else {
      printf("Unknown address resolved to '%s'\n", name);
    }

    free(addr);
    free(name);

    wait_for_addr = false;
    if ( ! wait_for_name && ! wait_for_addr )  at->cancel();
  }

  virtual void name_resolution_failed(char *name)
  {
    printf("Could not resolve '%s'\n", name);
    free(name);

    wait_for_name = false;
    if ( ! wait_for_name && ! wait_for_addr )  at->cancel();
  }


  virtual void address_resolution_failed(struct sockaddr_in *addr, socklen_t addrlen)
  {
    free(addr);

    wait_for_addr = false;
    if ( ! wait_for_name && ! wait_for_addr )  at->cancel();
  }


 private:
  AvahiThread *at;
  ArgumentParser *argp;

  bool wait_for_name;
  bool wait_for_addr;
};

int
main(int argc, char **argv)
{
  ArgumentParser *argp = new ArgumentParser(argc, argv, "n:a:");

  if ( ! argp->has_arg("n") && ! argp->has_arg("a") ) {
    printf("Usage: %s [-n name] [-a address]\n\n", argv[0]);
    delete argp;
    return -1;
  }

  try {
    QAAvahiResolverMain m(argp);
    SignalManager::register_handler(SIGINT, &m);

    m.run();

  } catch (Exception &e) {
    e.print_trace();
  }

  SignalManager::finalize();
  delete argp;
}

/// @endcond
