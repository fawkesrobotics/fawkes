//     A* Collision Avoidance Algorithm by Stefan Jacobs
//     Copyright (C) 2002  Stefan Jacobs <Stefan_J@gmx.de>
//
//     This program is free software; you can redistribute it and/or modify
//     it under the terms of the GNU General Public License as published by
//     the Free Software Foundation; either version 2 of the License, or
//     (at your option) any later version.
//
//     This program is distributed in the hope that it will be useful,
//     but WITHOUT ANY WARRANTY; without even the implied warranty of
//     MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
//     GNU General Public License for more details.
//
//     You should have received a copy of the GNU General Public License
//     along with this program; if not, write to the Free Software
//     Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
//


/*
  ©º°¨¨°º©º°¨¨°º©©º°¨¨°º©©º°¨¨°º©©º°¨¨°º©©º°¨¨°º©©º°¨¨°º©©º°¨¨°º©©º°¨¨°º©º°¨¨°º©
  ©                                                                            ©
  ©                                            ####   ####           .-""-.    ©
  ©       # #                             #   #    # #    #         /[] _ _\   ©
  ©       # #                                 #    # #             _|_o_LII|_  ©
  © ,###, # #  ### ## ## ##   ###  ## ##  #   #    # #       ###  / | ==== | \ ©
  © #   # # # #   # ## ## #  #   #  ## #  #   ###### #      #     |_| ==== |_| ©
  © #   # # # ####  #  #  #  #   #  #  #  #   #    # #      ####   ||" ||  ||  ©
  © #   # # # #     #  #  #  #   #  #  #  #   #    # #    #    #   ||LI  o ||  ©
  © '###'# # # #### #  #  ##  ### # #  ## ## #      # ####  ###    ||'----'||  ©
  ©                                                               /__|    |__\ ©
  ©                                                                            ©
  ©º°¨¨°º©º°¨¨°º©©º°¨¨°º©©º°¨¨°º©©º°¨¨°º©©º°¨¨°º©©º°¨¨°º©©º°¨¨°º©©º°¨¨°º©º°¨¨°º©
*/


/* ******************************************************************** */
/*                                                                      */
/* $Id$          */
/*                                                                      */
/* Description: This is the colli implementation of a collection e      */
/*                of fast ellipses.                                     */
/*                                                                      */
/* Author:   Stefan Jacobs                                              */
/* Contact:  <Stefan_J@gmx.de>                                          */
/*                                                                      */
/*                                                                      */
/*                                                                      */
/*                                                                      */
/* last modified: $Date$                          */
/*            by: $Author$                                    */
/*                                                                      */
/* ******************************************************************** */

#ifndef _COLLI_SEARCH_ELLIPSE_MAP_H_
#define _COLLI_SEARCH_ELLIPSE_MAP_H_

#include "ellipse.h"

#include <vector>
#include <map>

namespace fawkes
{
#if 0 /* just to make Emacs auto-indent happy */
}
#endif

class CEllipseMap
{
 public:
  //  CEllipseMap( int max_radius_width, int max_radius_height, int robocup_mode );
  CEllipseMap( );
  ~CEllipseMap() { m_mEllipses.clear(); }

  const std::vector< int > GetEllipse( int width, int height, int robocup_mode );

 private:
  std::map< unsigned int, CFastEllipse * > m_mEllipses;

};


//inline CEllipseMap::CEllipseMap( int max_radius_width, int max_radius_height,
//         int robocup_mode )

inline
CEllipseMap::CEllipseMap()
{
  //   for ( unsigned int x = 0; x < (unsigned int)max_radius_width; x++ )
  //     {
  //       for ( unsigned int y = 0; y < (unsigned int)max_radius_height; y++ )
  //  {
  //    CFastEllipse * ellipse = new CFastEllipse( x, y, robocup_mode );

  //    // What it does is the following: x * 2^16 + y. This is unique,
  //    //   because first it does a bit shift for 16 bits, and adds (or)
  //    //   afterwards a number that is smaller tham 16 bits!
  //    unsigned int key = (x << 16) | y;
  //    ellipse->SetKey( key );
  //    m_mEllipses[ key ] = ellipse;
  //  }
  //     }
}


inline const std::vector< int >
CEllipseMap::GetEllipse( int width, int height, int robocup_mode )
{
  unsigned int key = ((unsigned int)width << 16) | (unsigned int)height;

  std::map< unsigned int, CFastEllipse * >::iterator p = m_mEllipses.find( key );
  if ( p == m_mEllipses.end() ) {
    // ellipse nicht gefunden!
    CFastEllipse * ellipse = new CFastEllipse( width, height, robocup_mode );
    ellipse->SetKey( key );
    m_mEllipses[ key ] = ellipse;
    return ellipse->GetEllipse();

  } else {
    // ellipse in p gefunden
    return m_mEllipses[ key ]->GetEllipse();
  }
}

} // namespace fawkes

#endif
