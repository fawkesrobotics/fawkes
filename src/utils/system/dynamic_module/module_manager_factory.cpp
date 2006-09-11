
/***************************************************************************
 *  module_manager_factory.h - factory for module managers
 *
 *  Generated: Sun Sep 10 15:48:23 2006
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

#include <utils/system/dynamic_module/module_manager_factory.h>
#include <utils/system/dynamic_module/module_manager_template.h>
#include <utils/system/dynamic_module/module_dl.h>

ModuleManager *
ModuleManagerFactory::getInstance(ModuleManagerType mmt, std::string module_base_dir)
{
  switch (mmt) {
  case MMT_DL:
    return new ModuleManagerTemplate<ModuleDL>(module_base_dir);
  default:
    return NULL;
  }
}
