
/***************************************************************************
 *  net.cpp - Generic network tool
 *
 *  Created: Fri Nov 16 10:27:57 2007
 *  Copyright  2005-2009  Tim Niemueller [www.niemueller.de]
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

#include <fvutils/net/fuse.h>
#include <fvutils/net/fuse_client.h>
#include <fvutils/net/fuse_client_handler.h>
#include <fvutils/net/fuse_message.h>
#include <fvutils/net/fuse_image_content.h>
#include <fvutils/net/fuse_lut_content.h>
#include <fvutils/net/fuse_imagelist_content.h>
#include <fvutils/net/fuse_lutlist_content.h>
#include <fvutils/writers/fvraw.h>
#include <fvutils/color/colorspaces.h>
#include <fvutils/colormap/yuvcm.h>
#include <fvutils/colormap/cmfile.h>

#include <core/threading/mutex.h>
#include <core/threading/wait_condition.h>
#include <core/exceptions/software.h>
#include <utils/system/argparser.h>
#include <utils/system/console_colors.h>

#include <netcomm/service_discovery/browse_handler.h>
#ifdef HAVE_AVAHI
#include <netcomm/dns-sd/avahi_thread.h>
#endif

// for inet_ntop
#include <arpa/inet.h>
#include <netinet/in.h>
#include <cstring>
#include <cstdlib>
#include <cstdio>

using namespace fawkes;
using namespace firevision;

/** FireVision Network Tool */
class FireVisionNetworkTool
  : public FuseClientHandler,
    public ServiceBrowseHandler
{
 public:
  /** Constructor.
   * @param argp argument parser
   */
  FireVisionNetworkTool(ArgumentParser *argp)
  {
    __argp = argp;
    __exploring = false;
    __explore_waitcond = NULL;
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
  fuse_connection_died() throw()
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
	if ( ic->format() == FUSE_IF_RAW ) {
	  FvRawWriter *w = new FvRawWriter(__file, ic->pixel_width(), ic->pixel_height(),
					   (colorspace_t)ic->colorspace(), ic->buffer());
	  w->write();
	  delete w;
	} else if ( ic->format() == FUSE_IF_JPEG ) {
	  FILE *f = fopen(__file, "w");
	  if (fwrite(ic->buffer(), ic->buffer_size(), 1, f) == 0) {
	    printf("Failed to write data to file");
	  }
	  fclose(f);
	} else {
	  printf("Image of unknown format (%u) received.\n", ic->format());
	}
	delete ic;
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
	    printf("  %s (%u x %u, %s)\n", tmp, ntohl(ii->width), ntohl(ii->height),
		   colorspace_to_string((colorspace_t)ntohs(ii->colorspace)));
	  }
	} else {
	  printf("No images available\n");
	}
      delete ilc;
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
	    printf("  %s (%u x %u x %u, %u bpc)\n", tmp,
		   ntohl(li->width), ntohl(li->height),
		   ntohl(li->depth), ntohl(li->bytes_per_cell));
	  }
	} else {
	  printf("No lookup tables available\n");
	}
	delete llc;
      } catch (Exception &e) {
	printf("Received message cannot be casted to FuseImageListMessage\n");
	e.print_trace();
      }
      __client->cancel();
      break;

    case FUSE_MT_LUT:
      // we got a LUT, save it to the given file
      try {
	FuseLutContent *lc = m->msgc<FuseLutContent>();
	// Currently we expect colormaps, so make sure we get sensible dimensions
	if ( lc->width() != 256 ) {
	  printf("Invalid dimensions for LUT received, colormap width %u != 256", lc->width());
	} else if ( lc->height() != 256 ) {
	  printf("Invalid dimensions for LUT received, colormap height %u != 256", lc->height());
	} else if ( lc->depth() > 256 ) {
	  printf("Invalid dimensions for LUT received, colormap depth %u > 256", lc->depth());
	} else {
	  try {
	    YuvColormap yuvcm(lc->depth());
	    yuvcm.set(lc->buffer());
	    ColormapFile cmf;
	    cmf.add_colormap(&yuvcm);
	    cmf.write(__file);
	  } catch (Exception &e) {
	    e.append("Failed to save colormap");
	    e.print_trace();
	  }
	}
	delete lc;
      } catch (Exception &e) {
	printf("Received message cannot be casted to FuseLutMessage\n");
	e.print_trace();
      }
      __client->cancel();
      break;

    case FUSE_MT_SET_LUT_SUCCEEDED:
      {
	FUSE_lutdesc_message_t *lutdesc = m->msg<FUSE_lutdesc_message_t>();
	char lut_id[LUT_ID_MAX_LENGTH + 1];
	lut_id[LUT_ID_MAX_LENGTH] = 0;
	strncpy(lut_id, lutdesc->lut_id, LUT_ID_MAX_LENGTH);
	printf("LUT %s has been uploaded successfully.\n", lut_id);
	__client->cancel();
      }
      break;

    case FUSE_MT_SET_LUT_FAILED:
      {
	FUSE_lutdesc_message_t *lutdesc = m->msg<FUSE_lutdesc_message_t>();
	char lut_id[LUT_ID_MAX_LENGTH + 1];
	lut_id[LUT_ID_MAX_LENGTH] = 0;
	strncpy(lut_id, lutdesc->lut_id, LUT_ID_MAX_LENGTH);
	printf("LUT upload of %s has failed.\n", lut_id);
	__client->cancel();
      }
      break;

    default:
      printf("Unhandled message of type %u received\n", m->type());
      __client->cancel();
      break;
    }
  }


  virtual void all_for_now()
  {
    printf("All for now\n");
    __explore_mutex->lock();
    __explore_waitcond->wake_all();
    __explore_mutex->unlock();
  }

  virtual void cache_exhausted()
  {
  }

  virtual void browse_failed(const char *name,
			     const char *type,
			     const char *domain)
  {
    printf("Browsing for %s failed\n", type);
  }

  virtual void service_added(const char *name,
			     const char *type,
			     const char *domain,
			     const char *host_name,
			     const char *interface,
			     const struct sockaddr *addr,
			     const socklen_t addr_size,
			     uint16_t port,
			     std::list<std::string> &txt,
			     int flags
			     )
  {
    struct sockaddr_in *s;
    if ( addr_size == sizeof(struct sockaddr_in) ) {
      s = (struct sockaddr_in *)addr;
    } else {
      printf("%s socket data not IPv4, ignoring\n", name);
      return;
    }

    char addrp[INET_ADDRSTRLEN];
    inet_ntop(AF_INET, &(s->sin_addr), addrp, sizeof(addrp));
    printf("Found %s%s%s (%s/%s on %hu), querying\n",
	   c_blue, name, c_normal, host_name, addrp, port);

    __client = new FuseClient(host_name, port, this);
    __client->connect();
    __client->start();
    __client->wait_greeting();
    show_all();
    __client->join();
    delete __client;

    printf("\n");
  }

  virtual void service_removed(const char *name,
			       const char *type,
			       const char *domain)
  {
  }

  /** Print usage message. */
  void
  print_usage()
  {
    printf("Usage: %s -i/-c/-C/-s/-e [-n host[:port]/id file]\n"
	   "  -i             Get image\n"
	   "  -j             Get JPEG-compressed image\n"
	   "  -c             Get colormap\n"
	   "  -C             Set colormap from file\n"
	   "  -s             Show available images and LUTs\n"
	   "  -e             Explore network. Will query all instances of Fountain\n"
	   "                 found on the network for all available images and LUTs.\n"
	   "  -n net_string  Open network camera, the camera string is of the form\n"
	   "                 host[:port]/id. You have to specify at least the host\n"
	   "                 and the id, the port is optional and defaults to 5000\n"
	   "                 Depending on the operation id is the image or the LUT ID\n"
	   "  file           File to write incoming data to or to read data to send from\n",
	   __argp->program_name());
  }


  /** Request image.
   * @param image_id Image ID.
   * @param jpeg if true JPEG images are requested, raw images otherwise
   */
  void
  get_image(const char *image_id, bool jpeg)
  {
    FUSE_imagereq_message_t *idm = (FUSE_imagereq_message_t *)malloc(sizeof(FUSE_imagereq_message_t));
    memset(idm, 0, sizeof(FUSE_imagereq_message_t));
    strncpy(idm->image_id, image_id, IMAGE_ID_MAX_LENGTH-1);
    idm->format = (jpeg ? FUSE_IF_JPEG : FUSE_IF_RAW);
    __client->enqueue(FUSE_MT_GET_IMAGE, idm, sizeof(FUSE_imagereq_message_t));
  }

  /** Request LUT.
   * @param lut_id LUT ID.
   */
  void
  get_colormap(const char *lut_id)
  {
    FUSE_lutdesc_message_t *ldm = (FUSE_lutdesc_message_t *)malloc(sizeof(FUSE_lutdesc_message_t));
    memset(ldm, 0, sizeof(FUSE_lutdesc_message_t));
    strncpy(ldm->lut_id, lut_id, LUT_ID_MAX_LENGTH-1);
    __client->enqueue(FUSE_MT_GET_LUT, ldm, sizeof(FUSE_lutdesc_message_t));
  }

  /** Upload LUT.
   * @param lut_id LUT ID.
   */
  void
  set_colormap(const char *lut_id)
  {
    ColormapFile cmf;
    cmf.read(__file);
    Colormap *cm = cmf.get_colormap();
    FuseLutContent *lc = new FuseLutContent(lut_id, cm->get_buffer(),
					    cm->width(), cm->height(), cm->depth(),
					    /* bytes per cell */ 1);
    delete cm;

    __client->enqueue(new FuseNetworkMessage(FUSE_MT_SET_LUT, lc));
  }

  /** Show all images and LUTs. */
  void
  show_all()
  {
    __client->enqueue(FUSE_MT_GET_IMAGE_LIST);
    __client->enqueue(FUSE_MT_GET_LUT_LIST);
  }

  /** Explore network.
   * This will query via service discovery for all Fountain instances on the local
   * network. It will then connect to each of these and query them for existing images
   * and lookup tables.
   */
  void
  explore_network()
  {
#ifdef HAVE_AVAHI
    __exploring = true;
    __explore_mutex = new Mutex();
    __explore_waitcond = new WaitCondition(__explore_mutex);

    __explore_mutex->lock();

    __avahi_thread = new AvahiThread();
    __avahi_thread->start();

    __avahi_thread->watch_service("_fountain._tcp", this);

    __explore_waitcond->wait();
    delete __explore_waitcond;
    __explore_mutex->unlock();
    delete __explore_mutex;
    __avahi_thread->cancel();
    __avahi_thread->join();
    delete __avahi_thread;
#else
    printf("\nExploration is not available because Avahi support is missing. "
	   "Install avahi-devel and recompile.\n\n");
#endif
  }

  /** Run. */
  void
  run()
  {
    if ( __argp->has_arg("h") ) {
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
      int port_num = 2208;
      char *hostport;
      
      hostport = strtok_r(net_string, "/", &save_ptr);
      id = strtok_r(NULL, "", &save_ptr);

      if ( strchr(hostport, ':') != NULL ) {
	host = strtok_r(hostport, ":", &save_ptr);
	port = strtok_r(NULL, "", &save_ptr);
      } else {
	host = hostport;
      }

      if ( port != NULL ) {
	port_num = atoi(port);
	if ( (port_num < 0) || (port_num > 0xFFFF) ) {
	  throw OutOfBoundsException("Invalid port", port_num, 0, 0xFFFF);
	}
      }

      if (__argp->has_arg("i") || __argp->has_arg("j") ||
	  __argp->has_arg("c") || __argp->has_arg("C")) {
	if ( __argp->num_items() == 0 ) {
	  print_usage();
	  printf("\nFile name missing\n\n");
	  exit(1);
	} else {
	  __file = __argp->items()[0];
	}

	if (id == NULL) {
	  print_usage();
	  printf("\nNo Image/LUT ID given, needed for -i/-c/-C\n\n");
	  exit(2);
	}
      }

      if ( ! __argp->has_arg("e") ) {
	__client = new FuseClient(host, port_num, this);
	__client->connect();
	__client->start();
	__client->wait_greeting();
      }

      if ( __argp->has_arg("i") ) {
	get_image(id, /* JPEG? */ false);
      } else if ( __argp->has_arg("j") ) {
	get_image(id, /* JPEG? */ true);
      } else if ( __argp->has_arg("c") ) {
	get_colormap(id);
      } else if ( __argp->has_arg("C") ) {
	set_colormap(id);
      } else if ( __argp->has_arg("s") ) {
	show_all();
      } else if ( __argp->has_arg("e") ) {
	explore_network();
      } else {
	print_usage();
	__client->cancel();
      }

      if ( ! __argp->has_arg("e") ) {
	__client->join();
	delete __client;
      }

      free(net_string);
    }
  }

private:
  ArgumentParser *__argp;
  FuseClient     *__client;

  const char     *__file;

  bool            __exploring;
  Mutex          *__explore_mutex;
  WaitCondition  *__explore_waitcond;

#ifdef HAVE_AVAHI
  AvahiThread    *__avahi_thread;
#endif
};


int
main(int argc, char **argv)
{
  ArgumentParser argp(argc, argv, "hn:icCsej");

  FireVisionNetworkTool *nettool = new FireVisionNetworkTool(&argp);
  nettool->run();
  delete nettool;

  return 0;
}
