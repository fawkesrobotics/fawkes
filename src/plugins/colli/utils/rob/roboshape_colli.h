
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

/** @class CRoboShape_Colli <plugins/colli/utils/rob/roboshape_colli.h>
 *  This class is mainly the same as the basic class with the difference
 *    that all data is precalculated or estimated.
 */

class CRoboShape_Colli : public RoboShape
{
 public:

  CRoboShape_Colli( const char * cfg_prefix,
                    Logger* logger,
                    Configuration* config,
                    int readings_per_degree = 1 ) throw (int);
 ~CRoboShape_Colli();

  ///\brief Returns the robots length for a specific angle.
  float GetRobotLengthforRad( float anglerad );

  ///\brief Returns the robots length for a specific angle.
  float GetRobotLengthforDegree( float angledeg );

 private:

  // precalculated robot size data
  std::vector< float > m_vRobotLength;

  unsigned int m_Resolution;
};



/* ************************************************************************************************* */
/*                            IMPLEMENTATION DETAILS, DO NOT CARE!                                   */
/* ************************************************************************************************* */

/** Constructor
 * @param cfg_prefix The prefix of the config node, where the roboshape values are found
 * @param logger Pointer to the fawkes logger
 * @param config Pointer to the fawkes configuration.
 * @param readings_per_degree Readings per degree constant (default=1)
 */
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

  /** Destructor */
  inline
CRoboShape_Colli::~CRoboShape_Colli()
{
  m_vRobotLength.clear();
}

/** Returns the robots length for a specific angle.
 * @param anglerad is the angle in radians.
 * @return the length in this direction.
 */
inline float
CRoboShape_Colli::GetRobotLengthforRad( float anglerad )
{
  return (this->GetRobotLengthforDegree( rad2deg( anglerad ) ));
}

/** Returns the robots length for a specific angle.
 * @param angledeg is the angle in degree.
 * @return the length in this direction.
 */
inline float
CRoboShape_Colli::GetRobotLengthforDegree( float angledeg )
{
  int number = (int)(angledeg*m_Resolution);
  return m_vRobotLength[number];
}

} // namespace fawkes

#endif
