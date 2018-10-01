
/***************************************************************************
 *  main.cpp - Fawkes network log view
 *
 *  Created: Sat Dec 15 01:57:20 2007 (after I5 xmas party)
 *  Copyright  2006-2007  Tim Niemueller [www.niemueller.de]
 *
 ****************************************************************************/

/*  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU Library General Public License for more details.
 *
 *  Read the full text in the LICENSE.GPL file in the doc directory.
 */

#include <netcomm/fawkes/client.h>
#include <netcomm/fawkes/client_handler.h>
#include <netcomm/fawkes/component_ids.h>
#include <network_logger/network_logger.h>
#include <logging/console.h>
#include <utils/system/signal.h>
#include <utils/system/argparser.h>

#include <cstring>
#include <cstdio>
#include <cstdlib>

using namespace fawkes;

/// @cond INTERNALS

class NetLogConsolePrinter
  : public FawkesNetworkClientHandler,
    public SignalHandler
{
 public:
  NetLogConsolePrinter(const char *hostport)
  {
    logger = new ConsoleLogger();
    quit = false;

    char *hp = strdup(hostport);
    const char *hostname = strtok(hp, ":");
    const char *portstr = strtok(NULL, "");
    int port = 1910;
    if ( portstr ) {
      port = atoi(portstr);
      if ( (port < 0) || ( port > 0xFFFF ) ) {
	printf("Invalid port given, must be in range [1:65535]. Using default 1910 instead\n");
	port = 1910;
      }
    }

    client = new FawkesNetworkClient(hostname, port);
    client->connect();
    client->register_handler(this, FAWKES_CID_NETWORKLOGGER);

    client->enqueue(new FawkesNetworkMessage(FAWKES_CID_NETWORKLOGGER,
					     NetworkLogger::MSGTYPE_SUBSCRIBE));
  }

  ~NetLogConsolePrinter()
  {
    delete logger;
    delete client;
  }

  void handle_signal(int signal)
  {
    quit = true;
    client->wake(FAWKES_CID_NETWORKLOGGER);
  }

  virtual void inbound_received(FawkesNetworkMessage *m,
				unsigned int id) throw()
  {
    if ( (m->cid() == FAWKES_CID_NETWORKLOGGER) &&
	 (m->msgid() == NetworkLogger::MSGTYPE_LOGMESSAGE) ) {
      NetworkLoggerMessageContent *content = m->msgc<NetworkLoggerMessageContent>();
      struct timeval t = content->get_time();
      logger->tlog(content->get_loglevel(), &t, content->get_component(), "%s", content->get_message());
    }
  }

  virtual void deregistered(unsigned int id) throw()
  {
    quit = true;
  }


  virtual void connection_died(unsigned int id) throw()
  {
    printf("Connection to host died. Aborting.\n");
    quit = true;
  }


  virtual void connection_established(unsigned int id) throw()
  {
  }


  void run()
  {
    while ( ! quit ) {
      client->wait(FAWKES_CID_NETWORKLOGGER);
    }
    client->disconnect();
  }


 private:
  FawkesNetworkClient *client;
  ConsoleLogger *logger;
  bool quit;
};
/// @endcond


void
print_usage(const char *program_name)
{
  printf("Usage: %s [hostname[:port]]\n", program_name);
}

int
main(int argc, char **argv)
{
  ArgumentParser argp(argc, argv, "h");

  if ( argp.has_arg("h") ) {
    print_usage(argv[0]);
    exit(0);
  }

  const char *hostport = (argp.num_items() > 0) ? argp.items()[0] : "localhost:1910";
  NetLogConsolePrinter printer(hostport);

  SignalManager::register_handler(SIGINT, &printer);
  printer.run();

}

