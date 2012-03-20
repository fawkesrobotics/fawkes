
/***************************************************************************
 *  naoqi_broker.h - Transport NaoQi broker from module to plugin
 *
 *  Created: Tue May 31 11:04:18 2011
 *  Copyright  2006-2011  Tim Niemueller [www.niemueller.de]
 *
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

#ifndef __PLUGINS_NAO_NAOQI_BROKER_H_
#define __PLUGINS_NAO_NAOQI_BROKER_H_

#include <alcore/alptr.h>
#include <alcommon/albroker.h>

/* This is a hack to achieve the following: when the nao plugin is loaded,
 * we want to get access to the main broker for shared access within Fawkes.
 * However, for some reason the ALBrokerManager singleton is different in
 * the naofawkes NaoQi module that embeds Fawkes, and the nao plugin that
 * runs inside Fawkes.
 * This single extern declaration is intended to transport the broker from
 * the NaoQi module to the Fawkes plugin.
 */
namespace fawkes {
  namespace naoqi {
    extern AL::ALPtr<AL::ALBroker> broker;
  }
}

#endif
