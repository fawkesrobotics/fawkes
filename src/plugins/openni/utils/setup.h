
/***************************************************************************
 *  setup.h - OpenNI utility methods: setup routines
 *
 *  Created: Thu Mar 24 10:21:31 2011
 *  Copyright  2006-2014  Tim Niemueller [www.niemueller.de]
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

#ifndef __PLUGINS_OPENNI_UTILS_SETUP_H_
#define __PLUGINS_OPENNI_UTILS_SETUP_H_

#include <core/exception.h>
#include <core/utils/lockptr.h>

#include <XnCppWrapper.h>
#include <string>

namespace fawkes {
  class Configuration;

  namespace openni {
#if 0 /* just to make Emacs auto-indent happy */
  }
}
#endif

void get_resolution(fawkes::Configuration *config,
		    unsigned int &width, unsigned int &height);

void setup_map_generator(xn::MapGenerator &generator,
			 fawkes::Configuration *config);

void setup_alternate_viewpoint(xn::Generator &gen, xn::Generator &target);
void setup_synchronization(xn::Generator &gen, xn::Generator &target);

void get_usb_info(xn::Generator &gen, unsigned short &vendor, unsigned short &product);

/** Find existing or create new node.
 * This method will first try to find an existing node of the given type.
 * If this fails, it tries to create a new node of the desired type (leaving
 * the choice of the implementation to the system.
 * @param openni context to use, note that the context must have been locked
 * outside of this method call!
 * @param type node type
 * @param node instance that will be initialized for the node type
 * @exception Exception thrown if an error occurs while trying to find or
 * create the node. It may contain enumeration errors.
 */
template<class ProdNodeClass>
void find_or_create_node(fawkes::LockPtr<xn::Context> &openni,
			 XnProductionNodeType type, ProdNodeClass *node)
{
  XnStatus st;
  if ((st = openni->FindExistingNode(type, *node)) != XN_STATUS_OK) {
    xn::EnumerationErrors errors;
    if (node->Create(*(openni.operator->()), 0, &errors) != XN_STATUS_OK) {
      fawkes::Exception e("Failed to create user generator (%s)",
			  xnGetStatusString(st));
      for (xn::EnumerationErrors::Iterator i = errors.Begin();
           i != errors.End(); ++i)
      {
        XnProductionNodeDescription d = i.Description();
        e.append("%s: %s/%s/%u.%u.%u.%u: %s",
                 xnProductionNodeTypeToString(d.Type),
                 d.strVendor, d.strName, d.Version.nMajor, d.Version.nMinor,
                 d.Version.nMaintenance, d.Version.nBuild,
                 xnGetStatusString(i.Error()));
      }

      throw e;
    }
  }
}

} // end namespace fawkes::openni
} // end namespace fawkes





#endif
