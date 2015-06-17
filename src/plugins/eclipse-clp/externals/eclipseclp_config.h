
/***************************************************************************
 *  eclipseclp_config.h - config wrapper for Eclipse-CLP
 *
 *  Created: Fri Jan 30 17:17:16 2015 +0100
 *  Copyright  2015  Gesche Gierse
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

#include <config/config.h>

namespace fawkes
{
class EclExternalConfig
{
private:
  /** Constructor. */
  EclExternalConfig();

  /** Constructor.
   * @param config config instance to use
   */
  EclExternalConfig(Configuration *config);
public:
  /** Destructor. */
  ~EclExternalConfig();

  static void create_initial_object(Configuration *config);
  static EclExternalConfig* instance();

  static Configuration* config_instance();

private:
  static EclExternalConfig *          m_instance;
  static Configuration *              m_config;
};
}

extern "C" int p_get_config_value();
