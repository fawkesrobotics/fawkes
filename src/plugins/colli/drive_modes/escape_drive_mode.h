
/***************************************************************************
 *  escape_drive_mode.h - Implementation of drive-mode "escape"
 *
 *  Created: Fri Oct 18 15:16:23 2013
 *  Copyright  2002  Stefan Jacobs
 *             2013  Bahram Maleki-Fard
 *             2014  Tobias Neumann
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

#ifndef __PLUGINS_COLLI_ESCAPE_DRIVE_MODE_H_
#define __PLUGINS_COLLI_ESCAPE_DRIVE_MODE_H_

#include "abstract_drive_mode.h"
#include "../utils/rob/roboshape_colli.h"

#include <utils/math/types.h>

#include <vector>

namespace fawkes
{
#if 0 /* just to make Emacs auto-indent happy */
}
#endif

class CEscapeDriveModule : public CAbstractDriveMode
{
 public:
  CEscapeDriveModule( Logger* logger, Configuration* config );
  ~CEscapeDriveModule();

  virtual void Update();

  void setLaserData( std::vector<fawkes::polar_coord_2d_t>& laser_points );

 private:

  std::vector<fawkes::polar_coord_2d_t> laser_points_;

  CRoboShape_Colli*  m_pRoboShape;

  /// Readings without robolength in it
  std::vector< float > m_vNormalizedReadings;
  std::vector< float > m_vFront, m_vBack;
  std::vector< float > m_vLeftFront,  m_vLeftBack;
  std::vector< float > m_vRightFront, m_vRightBack;


  /// absolute values are the maximum values. do not act faster!
  float m_MaxTranslation;
  float m_MaxRotation;


  void FillNormalizedReadings();
  void SortNormalizedReadings();

  bool CheckDanger( std::vector< float > readings );
  bool TurnLeftAllowed();
  bool TurnRightAllowed();
};

} // end namespace fawkes

#endif
