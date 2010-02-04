
/***************************************************************************
 *  example_plugin_netping.cpp - Fawkes example plugin network ping
 *
 *  Created: Tue May 08 18:14:34 2007
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
#include <utils/system/argparser.h>
#include <utils/system/signal.h>

#include <cstdio>
#include <cstdlib>

using namespace fawkes;

/** Example Plugin network ping tool
 * Small class that waits for a reply of the example plugin after a short
 * network message was sent.
 */
class ExamplePluginClientNetworkReceiver : public FawkesNetworkClientHandler
{
 public:
  /** Constructor. */
  ExamplePluginClientNetworkReceiver()
  {
    quit = false;
  }

  /** The handler got deregistered.
   * @param id the id of the calling client
   */
  virtual void deregistered(unsigned int id) throw()
  {
    printf("Got deregistered\n");
    quit = true;
  }

  /** Inbound mesage received.
   * @param m message
   * @param id the id of the calling thread
   */
  virtual void inbound_received(FawkesNetworkMessage *m,
				unsigned int id) throw()
  {
    if ( m->payload_size() == sizeof(unsigned int) ) {
      unsigned int *u = (unsigned int *)m->payload();
      printf("Received message of type %hu with payload u=%u\n", m->msgid(), *u);
    } else {
      printf("Received message of invalid size, ignoring\n");
    }
    quit = true;
  }

  virtual void connection_died(unsigned int id) throw()
  {
    printf("Connection died.\n");
    quit = true;
  }


  virtual void connection_established(unsigned int id) throw()
  {
    printf("Connection established\n");
  }


  /** Set to true if answer has been received or handler was deregistered.
   * False at object creation.
   */
  bool quit;
};

/** Config tool main.
 * @param argc argument count
 * @param argv arguments
 */
int
main(int argc, char **argv)
{
  ArgumentParser argp(argc, argv, "Hn:i:");

  FawkesNetworkClient *c = new FawkesNetworkClient("localhost", 1910);
  c->connect();

  ExamplePluginClientNetworkReceiver r;
  c->register_handler(&r, FAWKES_CID_EXAMPLE_PLUGIN);

  const char *tmp;
  unsigned int *u = (unsigned int *)malloc(sizeof(unsigned int));;
  unsigned int id = 1;
  if ( (tmp = argp.arg("n")) != NULL ) {
    int i = atoi(tmp);
    if ( i > 0 ) {
      *u = i;
    }
  }

  if ( (tmp = argp.arg("i")) != NULL ) {
    int i = atoi(tmp);
    if ( i > 0 ) {
      id = i;
    }
  }


  FawkesNetworkMessage *msg = new FawkesNetworkMessage(FAWKES_CID_EXAMPLE_PLUGIN, id, u, sizeof(unsigned int));
  c->enqueue(msg);

  while ( ! r.quit ) {
    c->wait(FAWKES_CID_EXAMPLE_PLUGIN);
  }

  c->deregister_handler(FAWKES_CID_EXAMPLE_PLUGIN);
  c->disconnect();
  delete c;

  return 0;
}

