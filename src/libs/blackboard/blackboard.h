
/***************************************************************************
 *  blackboard.h - BlackBoard plugin
 *
 *  Generated: Sat Sep 16 17:09:15 2006 (on train to Cologne)
 *  Copyright  2006  Tim Niemueller [www.niemueller.de]
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
 *  along with this program; if not, write to the Free Software Foundation,
 *  Inc., 51 Franklin Street, Fifth Floor, Boston, MA 02111-1307, USA.
 */

#ifndef __BLACKBOARD_BLACKBOARD_H_
#define __BLACKBOARD_BLACKBOARD_H_

#include <blackboard/interface_manager.h>

class BlackBoard
{
 public:
  BlackBoard();
  ~BlackBoard();

  BlackBoardInterfaceManager *  interface_manager();

 private:
  BlackBoardInterfaceManager *im;

};


#endif
