
/***************************************************************************
 *  qa_fuse.h - QA for FUSE
 *
 *  Created: Wed Nov 14 17:13:14 2007
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

/// @cond QA

#include <fvutils/net/fuse_server.h>
#include <fvutils/net/fuse_client.h>
// include <fvutils/net/fuse_message.h>

#include <cstdio>

int
main(int argc, char **argv)
{

  FuseServer *fs = new FuseServer(5000);
  FuseClient *fc = new FuseClient("127.0.0.1", 5000, NULL);

  fc->connect();
  fc->start();

  usleep(5000);

  printf("Cancel and join\n");
  fc->cancel();
  fc->join();

  printf("Delete\n");
  delete fc;
  delete fs;

  return 0;
}



/// @endcond
