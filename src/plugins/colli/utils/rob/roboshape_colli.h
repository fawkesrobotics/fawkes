//     Roboshape class for colli A* by Stefan Jacobs
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
/* $Id$       */
/*                                                                      */
/* Description: This is the roboshape colli class.                      */
/*                                                                      */
/* Author:   Stefan Jacobs                                              */
/* Contact:  <Stefan_J@gmx.de>                                          */
/*                                                                      */
/* DOC.: The special about this class is, that all data is calculated   */
/*        during initialization time. All data that is not precalculated*/
/*        is estimated.                                                 */
/*                                                                      */
/* last modified: $Date$                          */
/*            by: $Author$                                    */
/*                                                                      */
/* ******************************************************************** */


#ifndef _COLLI_UTILS_ROB_ROBOSHAPE_COLLI_H_
#define _COLLI_UTILS_ROB_ROBOSHAPE_COLLI_H_

#include "roboshape.h"

#include <utils/math/angle.h>

#include <cmath>
#include <vector>

namespace fawkes
{
#if 0 /* just to make Emacs auto-indent happy */
}
#endif

class Logger;
class Configuration;

/** My RoboShape Colli class.
 *  This class is mainly the same as the basic class with the difference
 *    that all data is precalculated or estimated.
 */
class CRoboShape_Colli : public RoboShape
{
 public:

  /// Constructor
  ///  First param is a file name
  ///  Second param is the readings per degree constant.
  CRoboShape_Colli( const char * cfg_prefix,
                    Logger* logger,
                    Configuration* config,
                    int readings_per_degree = 1 ) throw (int);

  /// Destructor
  ~CRoboShape_Colli();

  /** Returns the robots length for a specific angle.
   *  @param anglerad is the angle in radians.
   *  @return the length in this direction.
   */
  float GetRobotLengthforRad( float anglerad );

  /** Returns the robots length for a specific angle.
   *  @param angledeg is the angle in degree.
   *  @return the length in this direction.
   */
  float GetRobotLengthforDegree( float angledeg );

 private:

  // precalculated robot size data
  std::vector< float > m_vRobotLength;

  unsigned int m_Resolution;
};



/* ************************************************************************************************* */
/*                            IMPLEMENTATION DETAILS, DO NOT CARE!                                   */
/* ************************************************************************************************* */


inline
CRoboShape_Colli::CRoboShape_Colli( const char * cfg_prefix,
                                    Logger* logger,
                                    Configuration* config,
                                    int readings_per_degree ) throw (int)
 : RoboShape( cfg_prefix, logger, config)
{
  m_Resolution = readings_per_degree;
  for ( int i = 0; i < 360*readings_per_degree; i++ ) {
    float anglerad = (i / readings_per_degree) * M_PI / 180;
    m_vRobotLength.push_back( this->RoboShape::GetRobotLengthforRad( anglerad ) );
  }
}

inline
CRoboShape_Colli::~CRoboShape_Colli()
{
  m_vRobotLength.clear();
}

inline float
CRoboShape_Colli::GetRobotLengthforRad( float anglerad )
{
  return (this->GetRobotLengthforDegree( rad2deg( anglerad ) ));
}

inline float
CRoboShape_Colli::GetRobotLengthforDegree( float angledeg )
{
  int number = (int)(angledeg*m_Resolution);
  return m_vRobotLength[number];
}

} // namespace fawkes

#endif
