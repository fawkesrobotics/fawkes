
/***************************************************************************
 *  main.cpp - Fawkes RefBox Repeater
 *
 *  Created: Wed Apr 09 09:46:29 2008
 *  Copyright  2006-2008  Tim Niemueller [www.niemueller.de]
 *
 *  $Id$
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

#include <utils/system/argparser.h>

#include "refbox_state_sender.h"
#include "midsize.h"
#include "spl.h"

#include <cstdlib>
#include <cstdio>
#include <cstring>

using namespace fawkes;

void
print_usage(const char *program_name)
{
  printf("Usage: %s [-d] -l league -t team -g goal_color\n"
	 "  -d             Turn on debug mode (prints to stdout)\n"
	 "  -l league      Define league, may be one of midsize or spl\n"
	 "  -t team        Our team, either cyan or magenta\n"
	 "  -g goal_color  Our goal color, either blue or yellow\n"
	 "  -p port        UDP port to send to (default 2806)\n"
	 "  -m addr        Multicast address to send to (default 224.16.0.1)\n"
	 "  -k key         Encryption key (default AllemaniACs)\n"
	 "  -i iv          Encryption initialization vector (default AllemaniACs)\n",
	 program_name);
}

/** Config tool main.
 * @param argc argument count
 * @param argv arguments
 */
int
main(int argc, char **argv)
{
  ArgumentParser argp(argc, argv, "hdl:t:g:p:m:k:i:");

  if ( argp.has_arg("h") ) {
    print_usage(argv[0]);
    exit(0);
  }

  if ( ! argp.has_arg("l") ) {
    printf("You must give a league name.\n\n");
    print_usage(argv[0]);
    exit(1);
  }

  if ( ! argp.has_arg("t") ) {
    printf("You must give our team color.\n\n");
    print_usage(argv[0]);
    exit(2);
  }

  if ( ! argp.has_arg("g") ) {
    printf("You must give our goal color.\n\n");
    print_usage(argv[0]);
    exit(3);
  }

  worldinfo_gamestate_team_t our_team;
  worldinfo_gamestate_goalcolor_t our_goal;
  const char *addr = "224.16.0.1";
  const char *key  = "AllemaniACs";
  const char *iv   = "AllemaniACs";
  unsigned short int port = 2806;

  if ( strcmp(argp.arg("t"), "cyan") == 0 ) {
    our_team = TEAM_CYAN;
  } else if ( strcmp(argp.arg("t"), "magenta") == 0 ) {
    our_team = TEAM_MAGENTA;
  } else {
    printf("Invalid team '%s', must be one of 'cyan' and 'magenta'.\n\n", argp.arg("t"));
    print_usage(argv[0]);
    exit(4);
  }

  if ( strcmp(argp.arg("g"), "blue") == 0 ) {
    our_goal = GOAL_BLUE;
  } else if ( strcmp(argp.arg("g"), "yellow") == 0 ) {
    our_goal = GOAL_YELLOW;
  } else {
    printf("Invalid goal '%s', must be one of 'blue' and 'yellow'.\n\n", argp.arg("g"));
    print_usage(argv[0]);
    exit(5);
  }

  if ( argp.has_arg("m") ) {
    addr = argp.arg("m");
  }

  if ( argp.has_arg("k") ) {
    key = argp.arg("k");
  }

  if ( argp.has_arg("i") ) {
    iv = argp.arg("i");
  }

  if ( argp.has_arg("p") ) {
    port = atoi(argp.arg("p"));
  }

  printf("Sending to: %s:%u\n"
	 "Key: %s  IV: %s\n", addr, port, key, iv);

  RefBoxStateSender rss(addr, port, key, iv, argp.has_arg("d"));
  rss.set_team_goal(our_team, our_goal);

  printf("League: %s\n", argp.arg("l"));
  if ( strcmp(argp.arg("l"), "midsize") == 0 ) {
    MidsizeRefBoxRepeater mrr(rss, "127.0.0.1", 28097);
    mrr.run();
  } else if ( strcmp(argp.arg("l"), "spl") == 0 ) {
    SplRefBoxRepeater nrr(rss, "255.255.255.0", 3838);
    nrr.run();
  } else {
    printf("Invalid league name given.\n\n");
    print_usage(argv[0]);
    exit(2);
  }

  return 0;
}
