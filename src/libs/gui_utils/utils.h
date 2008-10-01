
/***************************************************************************
 *  utils.h - Obtain widget object from Glade file
 *
 *  Created: Tue Sep 23 13:38:20 2008
 *  Copyright  2008  Daniel Beck
 *
 *  $Id$
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

#ifndef __GUIUTILS_UTILS_H_
#define __GUIUTILS_UTILS_H_

#include <libglademm/xml.h>
#include <gtkmm.h>

namespace fawkes {

Gtk::Widget* get_widget( Glib::RefPtr<Gnome::Glade::Xml> ref_xml,
			 const char* widget_name );

}


#endif /* __GUIUTILS_GET_WIDGET_H_ */
