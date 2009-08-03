
/***************************************************************************
 *  qa_worldinfo_msgsizes.h - Worldinfo QA showing sizes of messages
 *
 *  Created: Wed Jun 13 18:43:00
 *  Copyright  2006-2008  Tim Niemueller [www.niemueller.de]
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


#include <netcomm/worldinfo/messages.h>

#include <cstdio>

using namespace fawkes;

int
main(int argc, char **argv)
{

  printf("sizeof(worldinfo_message_header_t) = %lu\n", (unsigned long int)sizeof(worldinfo_message_header_t));
  printf("sizeof(worldinfo_header_t) = %lu\n", (unsigned long int)sizeof(worldinfo_header_t));
  printf("sizeof(worldinfo_pose_message_t) = %lu\n", (unsigned long int)sizeof(worldinfo_pose_message_t));
  printf("sizeof(worldinfo_velocity_message_t) = %lu\n", (unsigned long int)sizeof(worldinfo_velocity_message_t));
  printf("sizeof(worldinfo_relballpos_message_t) = %lu\n", (unsigned long int)sizeof(worldinfo_relballpos_message_t));
  printf("sizeof(worldinfo_relballvelo_message_t) = %lu\n", (unsigned long int)sizeof(worldinfo_relballvelo_message_t));
  printf("sizeof(worldinfo_opppose_message_t) = %lu\n", (unsigned long int)sizeof(worldinfo_opppose_message_t));
  printf("sizeof(worldinfo_fat_message_t) = %lu\n", (unsigned long int)sizeof(worldinfo_fat_message_t));

  return 0;
}

