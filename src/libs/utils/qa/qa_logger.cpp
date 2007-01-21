
/***************************************************************************
 *  qa_logger.cpp - QA for Logger
 *
 *  Generated: Wed Jan 17 14:19:45 2007
 *  Copyright  2005-2007  Tim Niemueller [www.niemueller.de]
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
 *  along with this program; if not, write to the Free Software
 *  Foundation, Inc., 59 Temple Place - Suite 330, Boston, MA 02111-1307, USA.
 */

// Do not include in api reference
///@cond QA

#include <utils/logging/console.h>

int
main(int argc, char **argv)
{

  ConsoleLogger cl;

  Exception e("Test Exception");

  cl.log_debug("QA", "DEBUG test output %i", 1);
  cl.log_info("QA", "DEBUG test output %i", 2);
  cl.log_warn("QA", "DEBUG test output %i", 3);
  cl.log_error("QA", "DEBUG test output %i", 4);

  cl.log_debug("QA", e);
  cl.log_info("QA", e);
  cl.log_warn("QA", e);
  cl.log_error("QA", e);

  return 0;
}

/// @endcond
