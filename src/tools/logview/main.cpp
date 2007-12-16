
/***************************************************************************
 *  main.cpp - Fawkes network log view
 *
 *  Created: Sat Dec 15 01:57:20 2007 (after I5 xmas party)
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

#include <netcomm/fawkes/client.h>
#include <netcomm/fawkes/client_handler.h>
#include <netcomm/fawkes/component_ids.h>
#include <netcomm/utils/network_logger.h>
#include <utils/logging/console.h>
#include <utils/system/signal.h>

/// @cond INTERNALS

class NetLogConsolePrinter
  : public FawkesNetworkClientHandler,
    public SignalHandler
{
 public:
  NetLogConsolePrinter()
  {
    logger = new ConsoleLogger();
    quit = false;

    client = new FawkesNetworkClient("localhost", 1910);
    client->connect();
    client->start();
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

  virtual void inbound_received(FawkesNetworkMessage *m) throw()
  {
    if ( (m->cid() == FAWKES_CID_NETWORKLOGGER) &&
	 (m->msgid() == NetworkLogger::MSGTYPE_LOGMESSAGE) ) {
      NetworkLoggerMessageContent *content = m->msgc<NetworkLoggerMessageContent>();
      struct timeval t = content->get_time();
      /* Yes, it is risky to just use get_message() as format, but for now we are happy
       * and do not expect bad guys. To be fixed. */
      logger->tlog(content->get_loglevel(), &t, content->get_component(), content->get_message());
    }
  }

  virtual void deregistered() throw()
  {
    quit = true;
  }


  virtual void connection_died() throw()
  {
    printf("Connection to host died. Aborting.\n");
    quit = true;
  }


  virtual void connection_established() throw()
  {
  }


  void run()
  {
    while ( ! quit ) {
      client->wait(FAWKES_CID_NETWORKLOGGER);
    }
    client->cancel();
    client->join();
  }


 private:
  FawkesNetworkClient *client;
  ConsoleLogger *logger;
  bool quit;
};
/// @endcond


int
main(int argc, char **argv)
{
  NetLogConsolePrinter printer;

  SignalManager::register_handler(SIGINT, &printer);
  printer.run();

}

