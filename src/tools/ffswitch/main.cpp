
/***************************************************************************
 *  main.cpp - Fawkes switch toggler tool main
 *
 *  Created: Thu Dez 10 15:34:18 2015
 *  Copyright  2015  Gesche Gierse
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
#include <blackboard/remote.h>

#include <utils/system/argparser.h>

#include <interfaces/SwitchInterface.h>

#include <string>
#include <cstdlib>
#include <cstdio>

using namespace fawkes;

/** Print usage.
 * @param program_name program name
 */
void
print_usage(const char *program_name)
{
    printf("Usage: %s [-e interface_id|-d interface_id] [-r host[:port]]\n"
         "  -e         Send enable msg to the switch interface specified by interface_id\n"
         "  -d         Send disable msg to the switch interface specified by interface_id\n"
         "  -r host[:port] Remote host (and optionally port) to connect to\n\n",
         program_name);
}

int
main(int argc, char **argv)
{
  ArgumentParser argp(argc, argv, "he:d:r:");

  if ( argp.has_arg("h") ) {
    print_usage(argp.program_name());
    exit(0);
  }

  std::string host = "localhost";
  unsigned short int port = 1910;
  if ( argp.has_arg("r") ) {
    argp.parse_hostport("r", host, port);
  }

  FawkesNetworkClient *c = new FawkesNetworkClient(host.c_str(), port);
  try {
    c->connect();
  } catch( Exception &e ) {
    printf("Could not connect to host: %s\n", host.c_str());
    exit(1);
  }

  try {
    BlackBoard *bb = new RemoteBlackBoard(c);
    //SwitchInterface *sw_if = bb->open_for_reading<SwitchInterface>("Start");
    if (argp.has_arg("e")) {
      const char *switch_name = argp.arg("e");
      SwitchInterface *sw_if = bb->open_for_reading<SwitchInterface>(switch_name);
      //send enable msg
      SwitchInterface::EnableSwitchMessage *em = new SwitchInterface::EnableSwitchMessage();
      sw_if->msgq_enqueue(em);
      bb->close(sw_if);
    }else if (argp.has_arg("d")) {
      const char *switch_name = argp.arg("d");
      SwitchInterface *sw_if = bb->open_for_reading<SwitchInterface>(switch_name);
      //send disable msg
      SwitchInterface::DisableSwitchMessage *dm = new SwitchInterface::DisableSwitchMessage();
      sw_if->msgq_enqueue(dm);
      bb->close(sw_if);
    }

    delete bb;

  } catch (Exception &e) {
    printf("Error connecting to BlackBoard: %s\n", e.what());
    c->disconnect();
  }

  c->disconnect();
  delete c;

  return 0;
}
