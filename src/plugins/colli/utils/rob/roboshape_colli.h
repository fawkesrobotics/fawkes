
/***************************************************************************
 *  roboshape_colli.h - RoboShape class for colli with precalculated data
 *
 *  Created: Fri Oct 18 15:16:23 2013
 *  Copyright  2002  Stefan Jacobs
 *             2013  Bahram Maleki-Fard
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

#ifndef __PLUGINS_COLLI_UTILS_ROB_ROBOSHAPE_COLLI_H_
#define __PLUGINS_COLLI_UTILS_ROB_ROBOSHAPE_COLLI_H_

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
