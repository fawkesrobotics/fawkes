/***************************************************************************
 *  clips_pddl_parser_thead.h - CLIPS feature for parsing PDDL domains
 *
 *  Created: Fri 16 Feb 2018 17:51:39 CET 17:51
 *  Copyright  2018  Till Hofmann <hofmann@kbsg.rwth-aachen.de>
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

#ifndef __PLUGINS_CLIPS_PDDL_PARSER_THEAD_H_
#define __PLUGINS_CLIPS_PDDL_PARSER_THEAD_H_

#include <core/threading/thread.h>
#include <aspect/logging.h>
#include <plugins/clips/aspect/clips_feature.h>

class ClipsPddlParserThread
: public fawkes::Thread,
  public fawkes::LoggingAspect,
  public fawkes::CLIPSFeatureAspect
{
  public:
    ClipsPddlParserThread();
    //virtual ~ClipsPddlParserThread();
};
#endif /* !__PLUGINS_CLIPS_PDDL_PARSER_THEAD_H_ */
