
/***************************************************************************
 *  msl2010.cpp - Fawkes mid-size refbox 2010 protocol repeater
 *
 *  Created: Wed Apr 09 10:38:16 2008
 *  Copyright  2008  Stefan Schiffer [stefanschiffer.de]
 *             2010  Tim Niemueller [www.niemueller.de]
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

#include "msl2010.h"
#include <netcomm/socket/datagram_multicast.h>
#include <utils/logging/logger.h>

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


/** @class Msl2010RefBoxProcessor "processor/msl2010.h"
 * Mid-size league refbox repeater.
 * This class will communicate with the mid-size league refbox and derive matching
 * game states from the communiation stream and send this via the world info.
 * @author Stefan Schiffer
 */

/** Constructor.
 * @param logger logger for output
 * @param refbox_host refbox host
 * @param refbox_port refbox port
 */
Msl2010RefBoxProcessor::Msl2010RefBoxProcessor(Logger *logger,
					       const char *refbox_host,
					       unsigned short int refbox_port)
  : __name("Msl2010RefBoxProc")
{
  __logger = logger;
  __quit = false;
  __s = NULL;
  __score_cyan = __score_magenta = 0;
  __connection_died = false;

  __refbox_host = strdup(refbox_host);
  __refbox_port = refbox_port;

  do {
    reconnect();
  } while (! __s);
}


/** Destructor. */
Msl2010RefBoxProcessor::~Msl2010RefBoxProcessor()
{
  free(__refbox_host);
  __s->close();
  delete __s;
}


/** Reconnect to refbox. */
void
Msl2010RefBoxProcessor::reconnect()
{
  if ( __s ) {
    __s->close();
    delete __s;
  }
  __logger->log_info(__name, "Trying to connect to refbox at %s:%u",
		     __refbox_host, __refbox_port);
  try {
    __logger->log_info(__name, "Creating MulticastDatagramSocket");
    __s = new MulticastDatagramSocket(__refbox_host, __refbox_port, 2.3);
    //printf("set loop\n");
    __s->set_loop(true); // (re)receive locally sent stuff
    //printf("bind\n");
    __s->bind();
    //printf("bind done\n");
    
    // printf("check for data availability ...\n");
    // if ( !__s->available() ) {
    //   printf("... nothing to receive\n");
    // } else {
    //   printf("... data is available!\n");
    // }

    __connection_died = false;

  } catch (Exception &e) {
    delete __s;
    __s = NULL;
    //printf(".");
    fflush(stdout);
    usleep(500000);
  }

  __logger->log_info(__name, "Init done");
}


/** Process received string. */
void
Msl2010RefBoxProcessor::process_string(char *buf, size_t len)
{
  __logger->log_info(__name, "Received\n *****\n %s \n *****", buf);

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
    __logger->log_info(__name, "root-element name is '%s'", el->get_name().data() );

    const Node::NodeList nl = el->get_children();

    if( nl.size() == 0 ) {
      __logger->log_info(__name, "root has NO children!");
    }
    else {
      //printf("root has %u children!\n", nl.size());

      for (Node::NodeList::const_iterator it = nl.begin(); it != nl.end(); ++it) {
	const Node* node = *it;
	__logger->log_info(__name, "1st level child name is '%s'", node->get_name().data() );

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
	  __logger->log_info(__name, "child has NO children!");
	}
	else {
	  //printf("child has %u children!\n", nl.size());
	  
	  for (Node::NodeList::const_iterator cit = cnl.begin(); cit != cnl.end(); ++cit) {
	    const Node*  cnode = *cit;
	    const Element* cel = dynamic_cast<const Element *>(cnode);
	    std::string cnodename(cnode->get_name().data());

	    __logger->log_info(__name, "2nd level child name is '%s'", cnode->get_name().data() );

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
	      __logger->log_info(__name, "RefBox cancelled last command");
 	    }
 	    else if( cnodename == REFBOX_GAMESTOP ) {
 	      _rsh->set_gamestate(GS_FROZEN, TEAM_BOTH);
 	    }
 	    else if( cnodename == REFBOX_GAMESTART ) {
 	      _rsh->set_gamestate(GS_PLAY, TEAM_BOTH);
 	    }
	    else if( cnodename == REFBOX_DROPPEDBALL ) {
	      _rsh->set_gamestate(GS_DROP_BALL, TEAM_BOTH);
	    }
	    else if( cnodename == REFBOX_GOAL_AWARDED ) {
	      // increment according to color
	      if( cteamcolor == REFBOX_TEAMCOLOR_CYAN ) {
		_rsh->set_score(++__score_cyan, __score_magenta);
	      } 
	      else if ( cteamcolor == REFBOX_TEAMCOLOR_MAGENTA ) {
		_rsh->set_score(__score_cyan, ++__score_magenta);
	      }
	      _rsh->set_gamestate(GS_FROZEN, TEAM_BOTH);
	    }
	    else if( cnodename == REFBOX_KICKOFF ) {
	      if( cteamcolor == REFBOX_TEAMCOLOR_CYAN ) {
		_rsh->set_gamestate(GS_KICK_OFF, TEAM_CYAN);
	      } 
	      else if ( cteamcolor == REFBOX_TEAMCOLOR_MAGENTA ) {
		_rsh->set_gamestate(GS_KICK_OFF, TEAM_MAGENTA);
	      }
	    }
	    else if( cnodename == REFBOX_PENALTY ) {
	      if( cteamcolor == REFBOX_TEAMCOLOR_CYAN ) {
		_rsh->set_gamestate(GS_PENALTY, TEAM_CYAN);
	      } 
	      else if ( cteamcolor == REFBOX_TEAMCOLOR_MAGENTA ) {
		_rsh->set_gamestate(GS_PENALTY, TEAM_MAGENTA);
	      }
	    }
	    else if( cnodename == REFBOX_CORNER ) {
	      if( cteamcolor == REFBOX_TEAMCOLOR_CYAN ) {
		_rsh->set_gamestate(GS_CORNER_KICK, TEAM_CYAN);
	      } 
	      else if ( cteamcolor == REFBOX_TEAMCOLOR_MAGENTA ) {
		_rsh->set_gamestate(GS_CORNER_KICK, TEAM_MAGENTA);
	      }
	    }
	    else if( cnodename == REFBOX_THROWIN ) {
	      if( cteamcolor == REFBOX_TEAMCOLOR_CYAN ) {
		_rsh->set_gamestate(GS_THROW_IN, TEAM_CYAN);
	      } 
	      else if ( cteamcolor == REFBOX_TEAMCOLOR_MAGENTA ) {
		_rsh->set_gamestate(GS_THROW_IN, TEAM_MAGENTA);
	      }
	    }
	    else if( cnodename == REFBOX_FREEKICK ) {
	      if( cteamcolor == REFBOX_TEAMCOLOR_CYAN ) {
		_rsh->set_gamestate(GS_FREE_KICK, TEAM_CYAN);
	      } 
	      else if ( cteamcolor == REFBOX_TEAMCOLOR_MAGENTA ) {
		_rsh->set_gamestate(GS_FREE_KICK, TEAM_MAGENTA);
	      }
	    }
	    else if( cnodename == REFBOX_GOALKICK ) {
	      if( cteamcolor == REFBOX_TEAMCOLOR_CYAN ) {
		_rsh->set_gamestate(GS_GOAL_KICK, TEAM_CYAN);
	      } 
	      else if ( cteamcolor == REFBOX_TEAMCOLOR_MAGENTA ) {
		_rsh->set_gamestate(GS_GOAL_KICK, TEAM_MAGENTA);
	      }
	    }
	    else if( cnodename == REFBOX_STAGE_CHANGED ) {
	      cattr = cel->get_attribute("newStage");
	      cstagetype = std::string( cattr->get_value().data() );
	      if( cstagetype == REFBOX_STAGETYPE_PREGAME ) {
		//
	      } else if( cstagetype == REFBOX_STAGETYPE_FIRSTHALF ) {
		_rsh->set_half(HALF_FIRST);
	      } else if( cstagetype == REFBOX_STAGETYPE_HALFTIME ) {
		_rsh->set_gamestate(GS_HALF_TIME, TEAM_BOTH);
	      } else if( cstagetype == REFBOX_STAGETYPE_SECONDHALF ) {
		_rsh->set_half(HALF_SECOND);
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
    __logger->log_info(__name, "root is NOT a valid element");
  }

}

void
Msl2010RefBoxProcessor::refbox_process()
{
  short pollrv = __s->poll(0, Socket::POLL_IN);
  do {
    
    if (pollrv == Socket::POLL_ERR) {
      __logger->log_warn(__name, "Polling socket failed");
    } else if (pollrv & Socket::POLL_IN) {
      char tmpbuf[1024];
      size_t bytes_read = __s->read(tmpbuf, sizeof(tmpbuf), /* read all */ false);
      __logger->log_debug(__name, "Read %zu bytes", bytes_read);
      if ( bytes_read == 0 ) {
	// seems that the remote has died, reconnect
	__connection_died = true;
      } else {
	tmpbuf[bytes_read] = '\0';
	process_string(tmpbuf, bytes_read);
      }
    }
    pollrv = __s->poll(0, Socket::POLL_IN);
  } while (pollrv & Socket::POLL_IN);
}

bool
Msl2010RefBoxProcessor::check_connection()
{
  if (__connection_died) {
    reconnect();
  }
  return ! __connection_died;
}
