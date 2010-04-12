
/***************************************************************************
 *  msl2010.cpp - Fawkes mid-size refbox 2010 protocol repeater
 *
 *  Created: Wed Apr 01 18:41:00 2010
 *  Copyright  2010  Stefan Schiffer [stefanschiffer.de]
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

#include <tools/refboxrep/msl2010.h>
#include <netcomm/socket/stream.h>
#include <netcomm/socket/datagram_multicast.h>

#include <cstring>
#include <cstdio>
#include <unistd.h>
#include <iostream>
#include <sstream>
#include <string>

#include <libxml++/libxml++.h>

using namespace fawkes;
using namespace xmlpp;


// REFBOX_CODES //////////////////////////

//static const char *  REFBOX_WELCOME          = "Welcome";
//static const char *  REFBOX_RECONNECT        = "Reconnect";

static const std::string REFBOX_EVENT               = "RefboxEvent";
static const std::string REFBOX_GAMEINFO            = "GameInfo";
static const std::string REFBOX_EVENT_REFEREE       = "Referee";
static const std::string REFBOX_EVENT_TEAMSETUP     = "TeamSetup";

static const std::string REFBOX_CANCEL              = "Cancel";

static const std::string REFBOX_GAMESTART           = "GameStart";
static const std::string REFBOX_GAMESTOP            = "GameStop";

static const std::string REFBOX_STAGE_CHANGED       = "StageChanged";
static const std::string REFBOX_STAGETYPE_PREGAME    = "preGame";
static const std::string REFBOX_STAGETYPE_FIRSTHALF  = "firstHalf";
static const std::string REFBOX_STAGETYPE_HALFTIME   = "halfTime";
static const std::string REFBOX_STAGETYPE_SECONDHALF = "secondHalf";
static const std::string REFBOX_STAGETYPE_SHOOTOUT   = "shootOut";
static const std::string REFBOX_STAGETYPE_ENDGAME    = "endGame";

static const std::string REFBOX_GOAL_AWARDED        = "GoalAwarded";
static const std::string REFBOX_GOAL_REMOVED        = "GoalRemoved";

static const std::string REFBOX_CARD_AWARDED        = "CardAwarded";
static const std::string REFBOX_CARD_REMOVED        = "CardRemoved";

static const std::string REFBOX_SUBSTITUTION        = "Substitution";
static const std::string REFBOX_PLAYER_OUT          = "PlayerOut";
static const std::string REFBOX_PLAYER_IN           = "PlayerIn";

static const std::string REFBOX_DROPPEDBALL         = "DroppedBall";
static const std::string REFBOX_KICKOFF             = "KickOff";
static const std::string REFBOX_FREEKICK            = "FreeKick";
static const std::string REFBOX_GOALKICK            = "GoalKick";
static const std::string REFBOX_THROWIN             = "ThrowIn";
static const std::string REFBOX_CORNER              = "Corner";
static const std::string REFBOX_PENALTY             = "Penalty";

static const std::string REFBOX_TEAMCOLOR_CYAN       = "Cyan";
static const std::string REFBOX_TEAMCOLOR_MAGENTA    = "Magenta";

static const std::string REFBOX_GOALCOLOR_YELLOW     = "yellow";
static const std::string REFBOX_GOALCOLOR_BLUE       = "blue";

static const std::string REFBOX_CARDCOLOR_YELLOW     = "yellow";
static const std::string REFBOX_CARDCOLOR_RED        = "red";


/** @class Msl2010RefBoxRepeater <tools/refboxrep/msl2010.h>
 * Mid-size league refbox repeater.
 * This class will communicate with the mid-size league refbox and derive matching
 * game states from the communiation stream and send this via the world info.
 * @author Stefan Schiffer
 */

/** Constructor.
 * @param rss refbox state sender
 * @param refbox_host refbox host
 * @param refbox_port refbox port
 * @param use_multicast use multicast connection (true by default)
 */
Msl2010RefBoxRepeater::Msl2010RefBoxRepeater(RefBoxStateSender &rss,
					     const char *refbox_host,
					     unsigned short int refbox_port,
					     const bool use_multicast )
  : __rss(rss)
{
  __quit = false;
  __s = NULL;
  __score_cyan = __score_magenta = 0;

  __refbox_host = strdup(refbox_host);
  __refbox_port = refbox_port;

  __use_multicast = use_multicast;

  reconnect();
}


/** Destructor. */
Msl2010RefBoxRepeater::~Msl2010RefBoxRepeater()
{
  free(__refbox_host);
  __s->close();
  delete __s;
}


/** Reconnect to refbox. */
void
Msl2010RefBoxRepeater::reconnect()
{
  if ( __s ) {
    __s->close();
    delete __s;
  }
  printf("Trying to connect to refbox at %s:%u\n", __refbox_host, __refbox_port);
  do {
    try {

      if( __use_multicast ) {
	
	printf("Creating MulticastDatagramSocket\n");
	__s = new MulticastDatagramSocket(__refbox_host, __refbox_port, 2.3);
	//printf("set loop\n");
	((MulticastDatagramSocket *)__s)->set_loop(true); // (re)receive locally sent stuff
	//printf("bind\n");
	((MulticastDatagramSocket *)__s)->bind();
	//printf("bind done\n");

	printf("check for data availability ...\n");
       if ( !__s->available() ) {
         printf("... nothing to receive\n");
       } else {
	 printf("... data is available!\n");
       }

      } 
      else {

	__s = new StreamSocket();
	__s->connect(__refbox_host, __refbox_port);

// 	char welcombuf[strlen(REFBOX_WELCOME) + 1];
// 	welcombuf[strlen(REFBOX_WELCOME)] = 0;
// 	char connectbuf[strlen(REFBOX_RECONNECT) + 1];
// 	connectbuf[strlen(REFBOX_RECONNECT)] = 0;
// 	__s->read(connectbuf, strlen(REFBOX_RECONNECT));
//	printf("Received welcome string: %s\n", connectbuf);

      }

    } catch (Exception &e) {
      delete __s;
      __s = NULL;
      printf("%s",e.what());
      printf("\n.");
      fflush(stdout);
      usleep(500000);
    }
  } while ( ! __s );

  printf("Connected.\n");
}


/** Process received string. */
void
Msl2010RefBoxRepeater::process_string(char *buf, size_t len)
{
  printf("Received\n *****\n %s \n *****\n", buf);

  std::istringstream iss( std::string(buf), std::istringstream::in);

  dom = new DomParser();
  //dom->set_validate();
  dom->set_substitute_entities();
  dom->parse_stream(iss);
  root = dom->get_document()->get_root_node();

  //printf( " root node:\n%s\n", root->get_name().data() );

  const Element * el = dynamic_cast<const Element *>(root);

  if ( el ) {
    /// valid element
    //printf("Is valid Element\n");
    printf("root-element name is '%s'\n", el->get_name().data() );

    const Node::NodeList nl = el->get_children();

    if( nl.size() == 0 ) {
      printf("root has NO children!\n");
    }
    else {
      //printf("root has %u children!\n", nl.size());

      for (Node::NodeList::const_iterator it = nl.begin(); it != nl.end(); ++it) {
	const Node* node = *it;
	printf("1st level child name is '%s'\n", node->get_name().data() );

	//if( node->get_name().data() == REFBOX_GAMEINFO ) {
	//
	//}
	//else if( node->get_name().data() == REFBOX_EVENT ) {
	//
	//}
	//else {
	//  printf(" unhandled RefboxMessage-type '%s'!\n", node->get_name().data() );
	//}
	
	const Node::NodeList cnl = node->get_children();

	if( cnl.size() == 0 ) {
	  printf("child has NO children!\n");
	}
	else {
	  //printf("child has %u children!\n", nl.size());
	  
	  for (Node::NodeList::const_iterator cit = cnl.begin(); cit != cnl.end(); ++cit) {
	    const Node*  cnode = *cit;
	    const Element* cel = dynamic_cast<const Element *>(cnode);
	    std::string cnodename(cnode->get_name().data());

	    printf("2nd level child name is '%s'\n", cnode->get_name().data() );

	    const Attribute* cattr;
	    std::string cteamcolor;
	    //std::string cgoalcolor;
	    //std::string ccardcolor;
	    std::string cstagetype;

	    if( cnodename == REFBOX_KICKOFF      || cnodename == REFBOX_FREEKICK     ||
		cnodename == REFBOX_GOALKICK     || cnodename == REFBOX_THROWIN      ||
		cnodename == REFBOX_CORNER       || cnodename == REFBOX_PENALTY      ||
		cnodename == REFBOX_GOAL_AWARDED || cnodename == REFBOX_GOAL_REMOVED ||
		cnodename == REFBOX_CARD_AWARDED || cnodename == REFBOX_CARD_REMOVED ||		
		cnodename == REFBOX_PLAYER_OUT   || cnodename == REFBOX_PLAYER_IN    ||
		cnodename == REFBOX_SUBSTITUTION ) 
	      {
		cattr = cel->get_attribute("team");
		cteamcolor = std::string( cattr->get_value().data() );
	      }

 	    if( cnodename == REFBOX_CANCEL ) {
 	      // refbox canceled last command
	      printf("RefBox cancelled last command\n");
 	    }
 	    else if( cnodename == REFBOX_GAMESTOP ) {
	      printf("sending command: REFBOX_GAMESTOP\n");
 	      __rss.set_gamestate(GS_FROZEN, TEAM_BOTH);
 	    }
 	    else if( cnodename == REFBOX_GAMESTART ) {
	      printf("sending command: REFBOX_GAMESTART\n");
 	      __rss.set_gamestate(GS_PLAY, TEAM_BOTH);
 	    }
	    else if( cnodename == REFBOX_DROPPEDBALL ) {
	      printf("sending command: REFBOX_DROPPEDBALL\n");
	      __rss.set_gamestate(GS_DROP_BALL, TEAM_BOTH);
	    }
	    else if( cnodename == REFBOX_GOAL_AWARDED ) {
	      // increment according to color
	      if( cteamcolor == REFBOX_TEAMCOLOR_CYAN ) {
		printf("sending command: REFBOX_TEAMCOLOR_CYAN\n");
		__rss.set_score(++__score_cyan, __score_magenta);
	      } 
	      else if ( cteamcolor == REFBOX_TEAMCOLOR_MAGENTA ) {
		printf("sending command: REFBOX_TEAMCOLOR_MAGENTA\n");
		__rss.set_score(__score_cyan, ++__score_magenta);
	      }
	      printf("sending command: GS_FROZEN\n");
	      __rss.set_gamestate(GS_FROZEN, TEAM_BOTH);
	    }
	    else if( cnodename == REFBOX_KICKOFF ) {
	      if( cteamcolor == REFBOX_TEAMCOLOR_CYAN ) {
		printf("sending command: GS_KICK_OFF, TEAM_CYAN\n");
		__rss.set_gamestate(GS_KICK_OFF, TEAM_CYAN);
	      } 
	      else if ( cteamcolor == REFBOX_TEAMCOLOR_MAGENTA ) {
		printf("sending command: GS_KICK_OFF, TEAM_MAGENTA\n");
		__rss.set_gamestate(GS_KICK_OFF, TEAM_MAGENTA);
	      }
	    }
	    else if( cnodename == REFBOX_PENALTY ) {
	      if( cteamcolor == REFBOX_TEAMCOLOR_CYAN ) {
		__rss.set_gamestate(GS_PENALTY, TEAM_CYAN);
	      } 
	      else if ( cteamcolor == REFBOX_TEAMCOLOR_MAGENTA ) {
		__rss.set_gamestate(GS_PENALTY, TEAM_MAGENTA);
	      }
	    }
	    else if( cnodename == REFBOX_CORNER ) {
	      if( cteamcolor == REFBOX_TEAMCOLOR_CYAN ) {
		__rss.set_gamestate(GS_CORNER_KICK, TEAM_CYAN);
	      } 
	      else if ( cteamcolor == REFBOX_TEAMCOLOR_MAGENTA ) {
		__rss.set_gamestate(GS_CORNER_KICK, TEAM_MAGENTA);
	      }
	    }
	    else if( cnodename == REFBOX_THROWIN ) {
	      if( cteamcolor == REFBOX_TEAMCOLOR_CYAN ) {
		__rss.set_gamestate(GS_THROW_IN, TEAM_CYAN);
	      } 
	      else if ( cteamcolor == REFBOX_TEAMCOLOR_MAGENTA ) {
		__rss.set_gamestate(GS_THROW_IN, TEAM_MAGENTA);
	      }
	    }
	    else if( cnodename == REFBOX_FREEKICK ) {
	      if( cteamcolor == REFBOX_TEAMCOLOR_CYAN ) {
		__rss.set_gamestate(GS_FREE_KICK, TEAM_CYAN);
	      } 
	      else if ( cteamcolor == REFBOX_TEAMCOLOR_MAGENTA ) {
		__rss.set_gamestate(GS_FREE_KICK, TEAM_MAGENTA);
	      }
	    }
	    else if( cnodename == REFBOX_GOALKICK ) {
	      if( cteamcolor == REFBOX_TEAMCOLOR_CYAN ) {
		__rss.set_gamestate(GS_GOAL_KICK, TEAM_CYAN);
	      } 
	      else if ( cteamcolor == REFBOX_TEAMCOLOR_MAGENTA ) {
		__rss.set_gamestate(GS_GOAL_KICK, TEAM_MAGENTA);
	      }
	    }
	    else if( cnodename == REFBOX_STAGE_CHANGED ) {
	      cattr = cel->get_attribute("newStage");
	      cstagetype = std::string( cattr->get_value().data() );
	      if( cstagetype == REFBOX_STAGETYPE_PREGAME ) {
		//
	      } else if( cstagetype == REFBOX_STAGETYPE_FIRSTHALF ) {
		__rss.set_half(HALF_FIRST);
	      } else if( cstagetype == REFBOX_STAGETYPE_HALFTIME ) {
		__rss.set_gamestate(GS_HALF_TIME, TEAM_BOTH);
	      } else if( cstagetype == REFBOX_STAGETYPE_SECONDHALF ) {
		__rss.set_half(HALF_SECOND);
	      } else if( cstagetype == REFBOX_STAGETYPE_SHOOTOUT ) {
		//
	      } else if( cstagetype == REFBOX_STAGETYPE_ENDGAME ) {
		//
	      }

	    }

	  } // end-for "child-node children list iteration"
	} // end-if "child-node has children"
      } // end-for "root children list iteration"
    } // end-if "root has children"
  }
  else {
    // throw RefBoxParserException("root is not an element");
    printf("root is NOT a valid element\n");
  }

  __rss.send();
}


/** Run.
 * Reads messages from the network, processes them and calls the refbox state sender.
 */
void
Msl2010RefBoxRepeater::run()
{
  //char tmpbuf[4096];
  char tmpbuf[1024];
  while ( ! __quit ) {
    size_t bytes_read = __s->read(tmpbuf, sizeof(tmpbuf), /* read all */ false);
    //size_t bytes_read = __s->read(tmpbuf, sizeof(tmpbuf), /* read all */ true );
    if ( bytes_read == 0 ) {
      // seems that the remote has died, reconnect
      printf("Connection died, reconnecting\n");
      reconnect();
    } else {
      printf("Received %zu bytes, processing ...\n", bytes_read);
      tmpbuf[bytes_read] = '\0';
      process_string(tmpbuf, bytes_read);
    }
  }
}
