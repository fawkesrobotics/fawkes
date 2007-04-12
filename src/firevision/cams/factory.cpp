
/***************************************************************************
 *  factory.cpp - Camera factory
 *
 *  Created: Wed Apr 11 15:37:45 2007
 *  Copyright  2005-2007  Tim Niemueller [www.niemueller.de]
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

#include <cams/factory.h>
#include <fvutils/system/camargp.h>

#include <string>

#ifdef HAVE_FIREWIRE_CAM
#include <cams/firewire.h>
#endif
#ifdef HAVE_LEUTRON_CAM
#include <cams/leutron.h>
#endif
#ifdef HAVE_FILELOADER_CAM
#include <cams/fileloader.h>
#endif
#ifdef HAVE_SHMEM_CAM
#include <cams/shmem.h>
#endif
#ifdef HAVE_NETWORK_CAM
#include <cams/net.h>
#endif
#ifdef HAVE_V4L_CAM
#include <cams/v4l.h>
#endif

using namespace std;

Camera *
CameraFactory::instance(const char *idents)
{
  CameraArgumentParser *cap = new CameraArgumentParser(idents);
  Camera *c = NULL;

#ifdef HAVE_FIREWIRE_CAM
  if ( cap->camid() == "firewire" ) {
    c = new FirewireCamera(cap);
  }
#endif
#ifdef HAVE_LEUTRON_CAM
  if ( cap->camid() == "leutron" ) {
    c = new LeutronCamera(cap);
  }
#endif
#ifdef HAVE_FILELOADER_CAM
  if ( cap->camid() == "file" ) {
    c = new FileLoader(cap);
 }
#endif
#ifdef HAVE_SHMEM_CAM
  if ( cap->camid() == "shmem" ) {
    c = new SharedMemoryCamera(cap);
 }
#endif
#ifdef HAVE_NETWORK_CAM
  if ( cap->camid() == "net" ) {
    c = new NetworkCamera(cap);
 }
#endif
#ifdef HAVE_V4L_CAM
  if ( cap->camid() == "v4l" ) {
    c = new V4LCamera(cap);
 }
#endif

  if ( c == NULL ) {
    throw UnknownCameraTypeException();
  }

  return c;
}
