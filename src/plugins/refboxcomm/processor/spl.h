
/***************************************************************************
 *  spl.h - Fawkes SPL refbox repeater
 *
 *  Created: Tue Jul 08 13:46:19 2008
 *  Copyright  2008-2010  Tim Niemueller [www.niemueller.de]
 *             2009       Tobias Kellner
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

#ifndef __TOOLS_REFBOXREP_SPL_H_
#define __TOOLS_REFBOXREP_SPL_H_

#include "processor.h"
#include <netcomm/worldinfo/enums.h>

#include <cstdlib>
#include <stdint.h>
#include <map>

namespace fawkes {
  class Logger;
  class DatagramSocket;
}

#define SPL_HEADER_SIZE 4
#define SPL_MAX_NUM_PLAYERS 11

#pragma pack(push,4)
/** SPL RefBox protocol robot info struct. */
typedef struct {
  uint16_t penalty;               /**< penalty state of the player */
  uint16_t secs_till_unpenalized; /**< estimate of time till unpenalised */
} spl_robotinfo_t;

/** SPL RefBox protocol team info struct. */
typedef struct {
  uint8_t  team_number;           /**< unique team number */
  uint8_t  team_color;            /**< colour of the team */
#ifdef USE_SPL_GC6
  uint16_t score;                 /**< team's score */
#else
  uint8_t  goal_color;            /**< colour of the goal */
  uint8_t  score;                 /**< team's score */
#endif
  spl_robotinfo_t players[SPL_MAX_NUM_PLAYERS];       /**< the team's players */
} spl_teaminfo_t;

/** SPL RefBox protocol game control struct. */
typedef struct {
  char      header[SPL_HEADER_SIZE];  /**< header to identify the structure */
  uint32_t  version;             /**< version of the data structure */
  uint8_t   players_per_team;    /**< The number of players on a team */
  uint8_t   state;               /**< state of the game (STATE_READY, STATE_PLAYING, etc.) */
  uint8_t   first_half;          /**< 1 = game in first half, 0 otherwise */
  uint8_t   kick_off_team;       /**< the next team to kick off */
  uint8_t   secondary_state;     /**< Extra state information - (STATE2_NORMAL, STATE2_PENALTYSHOOT, etc) */
  uint8_t   drop_in_team;        /**< team that caused last drop in */
  uint16_t  drop_in_time;        /**< number of seconds passed since the last drop in.  -1 before first dropin */
  uint32_t  secs_remaining;      /**< estimate of number of seconds remaining in the half */
  spl_teaminfo_t teams[2];       /**< Info about the teams */
} spl_gamecontrol_t;
#pragma pack(pop)

class SplRefBoxProcessor : public RefBoxProcessor
{
 public:
  SplRefBoxProcessor(fawkes::Logger *logger, unsigned short int broadcast_port,
                     unsigned int team_number, unsigned int player_number);
  ~SplRefBoxProcessor();

  void run();

  bool check_connection();
  void refbox_process();

 private:
  void process_struct(spl_gamecontrol_t *msg);

 private:
  fawkes::DatagramSocket *__s;
  fawkes::Logger         *__logger;

  bool __quit;

  uint16_t __penalty;
  uint8_t  __team_number;
  uint8_t  __player_number;
};

#endif
