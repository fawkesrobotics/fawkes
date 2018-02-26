/***************************************************************************
 *  clips_pddl_parser_thread.cpp - CLIPS feature for parsing PDDL domains
 *
 *  Created: Fri 16 Feb 2018 17:59:31 CET 17:59
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

#include "clips_pddl_parser_thread.h"
#include "clips_pddl_parser_feature.h"

/** @class ClipsPddlParserThread "clips_pddl_parser_thread.h"
 * Plugin thread that provides the pddl-parser CLIPS features.
 * @author Till Hofmann
 */

ClipsPddlParserThread::ClipsPddlParserThread()
  : Thread("ClipsPddlParserThread", Thread::OPMODE_WAITFORWAKEUP),
    CLIPSFeatureAspect(new PDDLCLIPSFeature(logger))
{
}
