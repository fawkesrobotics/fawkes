
/***************************************************************************
 *  module_manager_factory.h - factory class for module managers
 *
 *  Generated: Sun Sep 11 09:51:41 2006
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
 *  along with this program; if not, write to the Free Software
 *  Foundation, Inc., 59 Temple Place - Suite 330, Boston, MA 02111-1307, USA.
 */

#ifndef __UTILS_SYSTEM_DYNAMIC_MODULE_MODULE_MANAGER_FACTORY_H_
#define __UTILS_SYSTEM_DYNAMIC_MODULE_MANAGER_FACTORY_H_

#include <utils/system/dynamic_module/module_manager.h>

/** Class with just one static method to retrieve a module manager for the
 * specified type.
 * The main reason for this class is to hide the ModuleManagerTemplate
 * definition from ordinary processes.
 */
class ModuleManagerFactory
{
 public:

  typedef enum {
    MMT_DL = 1
  } ModuleManagerType;

  /** Retrieve an module manager instance of the desired type
   * @param mmt ModuleManagerType
   * @param module_base_dir The base directory where to look for modules, plainly copied
   * to the module manager, defaults to the empty string which is in most cases not desired
   * @return Returns an instance of a ModuleManager implementation. Delete after you are
   * done with this!
   */
  static ModuleManager * getInstance(ModuleManagerType mmt, std::string module_base_dir = "");
};

#endif
