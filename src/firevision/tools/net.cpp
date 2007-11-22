
/***************************************************************************
 *  net.cpp - Generic network tool
 *
 *  Created: Fri Nov 16 10:27:57 2007
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

#include <fvutils/net/fuse.h>
#include <fvutils/net/fuse_client.h>
#include <fvutils/net/fuse_client_handler.h>
#include <fvutils/net/fuse_message.h>
#include <fvutils/net/fuse_image_content.h>
#include <fvutils/net/fuse_lut_content.h>
#include <fvutils/net/fuse_imagelist_content.h>
#include <fvutils/net/fuse_lutlist_content.h>
#include <fvutils/writers/fvraw.h>

#include <core/exceptions/software.h>
#include <utils/system/argparser.h>

/** FireVision Network Tool */
class FireVisionNetworkTool : public FuseClientHandler
{
 public:
  /** Constructor.
   * @param argp argument parser
   */
  FireVisionNetworkTool(ArgumentParser *argp)
  {
    __argp = argp;
  }

  void
  fuse_invalid_server_version(uint32_t local_version,
			      uint32_t remote_version) throw()
  {
    printf("Invalid version received (local: %u, remote: %u)\n",
	   local_version, remote_version);
  }

  virtual void
  fuse_connection_established() throw()
  {
  }

  virtual void
  fuse_inbound_received(FuseNetworkMessage *m) throw()
  {
    // printf("Received message of type %u\n", m->type());

    switch (m->type() ) {
    case FUSE_MT_IMAGE:
      // we got an image, save it to the given file
      try {
	FuseImageContent *ic = m->msgc<FuseImageContent>();
	FvRawWriter *w = new FvRawWriter(__file, ic->pixel_width(), ic->pixel_height(),
					 (colorspace_t)ic->colorspace(), ic->buffer());
	w->write();
	delete w;
      } catch (Exception &e) {
	printf("Received message cannot be casted to FuseImageMessage\n");
	e.print_trace();
      }
      __client->cancel();
      break;
    case FUSE_MT_IMAGE_LIST:
      try {
	FuseImageListContent *ilc = m->msgc<FuseImageListContent>();
	if ( ilc->has_next() ) {
	  printf("Available images:\n");
	  while ( ilc->has_next() ) {
	    FUSE_imageinfo_t *ii = ilc->next();
	    char tmp[IMAGE_ID_MAX_LENGTH + 1];
	    tmp[IMAGE_ID_MAX_LENGTH] = 0;
	    strncpy(tmp, ii->image_id, IMAGE_ID_MAX_LENGTH);
	    printf("  %s\n", tmp);
	  }
	} else {
	  printf("No images available\n");
	}
      } catch (Exception &e) {
	printf("Received message cannot be casted to FuseImageListMessage\n");
	e.print_trace();
      }
      break;
    case FUSE_MT_LUT_LIST:
      try {
	FuseLutListContent *llc = m->msgc<FuseLutListContent>();
	if ( llc->has_next() ) {
	  printf("Available lookup tables:\n");
	  while ( llc->has_next() ) {
	    FUSE_lutinfo_t *li = llc->next();
	    char tmp[LUT_ID_MAX_LENGTH + 1];
	    tmp[LUT_ID_MAX_LENGTH] = 0;
	    strncpy(tmp, li->lut_id, LUT_ID_MAX_LENGTH);
	    printf("  %s\n", tmp);
	  }
	} else {
	  printf("No lookup tables available\n");
	}
      } catch (Exception &e) {
	printf("Received message cannot be casted to FuseImageListMessage\n");
	e.print_trace();
      }
      __client->cancel();
      break;
    case FUSE_MT_LUT:
      // we got an image, save it to the given file
      try {
	FuseLutContent *lc = m->msgc<FuseLutContent>();
	FILE *f = fopen(__file, "w");
	fwrite(lc->buffer(), lc->buffer_size(), 1, f);
	fclose(f);
      } catch (Exception &e) {
	printf("Received message cannot be casted to FuseLutMessage\n");
	e.print_trace();
      }
      __client->cancel();
      break;
    default:
      printf("Unhandled message of type %u received\n", m->type());
      break;
    }
  }

  /** Print usage message. */
  void
  print_usage()
  {
    printf("Usage: %s -i/-l/-L/-s -n host[:port]/id file\n"
	   "  -i             Get image\n"
	   "  -l             Get LUT\n"
	   "  -L             Set LUT\n"
	   "  -s             Show available images and LUTs\n"
	   "  -n net_string  Open network camera, the camera string is of the form\n"
	   "                 host[:port]/id. You have to specify at least the host\n"
	   "                 and the id, the port is optional and defaults to 5000\n"
	   "                 Depending on the operation id is the image or the LUT ID\n"
	   "  file           File to write incoming data to or to read data to send from\n",
	   __argp->program_name());
  }


  /** Request image.
   * @param image_id Image ID.
   */
  void
  get_image(const char *image_id)
  {
    FUSE_imagedesc_message_t *idm = (FUSE_imagedesc_message_t *)malloc(sizeof(FUSE_imagedesc_message_t));
    memset(idm, 0, sizeof(FUSE_imagedesc_message_t));
    strncpy(idm->image_id, image_id, IMAGE_ID_MAX_LENGTH);
    __client->enqueue(FUSE_MT_GET_IMAGE, idm, sizeof(FUSE_imagedesc_message_t));
  }

  /** Request LUT.
   * @param lut_id LUT ID.
   */
  void
  get_lut(const char *lut_id)
  {
    FUSE_lutdesc_message_t *ldm = (FUSE_lutdesc_message_t *)malloc(sizeof(FUSE_lutdesc_message_t));
    memset(ldm, 0, sizeof(FUSE_lutdesc_message_t));
    strncpy(ldm->lut_id, lut_id, LUT_ID_MAX_LENGTH);
    __client->enqueue(FUSE_MT_GET_LUT, ldm, sizeof(FUSE_lutdesc_message_t));
  }

  /** Upload LUT.
   * @param lut_id LUT ID.
   */
  void
  set_lut(const char *lut_id)
  {
    printf("This is currently not supported\n");
    __client->cancel();
  }

  /** Show all images and LUTs. */
  void
  show_all()
  {
    __client->enqueue(FUSE_MT_GET_IMAGE_LIST);
    __client->enqueue(FUSE_MT_GET_LUT_LIST);
  }

  /** Run. */
  void
  run()
  {
    if ( __argp->has_arg("H") ) {
      print_usage();
      exit(0);
    } else {
      char *net_string;
      if ( __argp->has_arg("n") ) {
	net_string = strdup(__argp->arg("n"));
      } else {
	net_string = strdup("localhost");
      }
      char *id = NULL, *host = NULL, *port = NULL, *save_ptr = NULL;
      int port_num = 5000;
      
      if ( strchr(net_string, ':') != NULL ) {
	host = strtok_r(net_string, ":", &save_ptr);
	port = strtok_r(NULL, "/", &save_ptr);
      } else {
	host = strtok_r(net_string, "/", &save_ptr);
      }
      id = strtok_r(NULL, "", &save_ptr);
    
      if ( port != NULL ) {
	port_num = atoi(port);
	if ( (port_num < 0) || (port_num > 0xFFFF) ) {
	throw OutOfBoundsException("Invalid port", port_num, 0, 0xFFFF);
	}
      }

      if (__argp->has_arg("i") || __argp->has_arg("l") || __argp->has_arg("L")) {
	if ( __argp->num_items() == 0 ) {
	  print_usage();
	  printf("\nFile name missing\n\n");
	  exit(1);
	} else {
	  __file = __argp->items()[0];
	}

	if (id == NULL) {
	  print_usage();
	  printf("\nNo Image ID given, needed for -i/-l/-L\n\n");
	  exit(2);
	}
      }

      __client = new FuseClient(host, port_num, this);
      __client->connect();
      __client->start();
 
      if ( __argp->has_arg("i") ) {
	get_image(id);
      } else if ( __argp->has_arg("l") ) {
	get_lut(id);
      } else if ( __argp->has_arg("L") ) {
	set_lut(id);
      } else if ( __argp->has_arg("s") ) {
	show_all();
      } else {
	print_usage();
	__client->cancel();
      }

      __client->join();
      delete __client;
    }
  }

private:
  ArgumentParser *__argp;
  FuseClient     *__client;

  const char     *__file;
};


int
main(int argc, char **argv)
{
  ArgumentParser argp(argc, argv, "Hn:ilLs");

  FireVisionNetworkTool *nettool = new FireVisionNetworkTool(&argp);
  nettool->run();

  return 0;
}
