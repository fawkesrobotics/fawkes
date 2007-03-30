
/***************************************************************************
 *  fuse_server_tcp.h - network image transport server TCP implementation
 *
 *  Generated: Tue Jan 10 11:04:55 2006
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

#ifndef __FIREVISION_FVUTILS_NET_FUSE_SERVER_TCP_H_
#define __FIREVISION_FVUTILS_NET_FUSE_SERVER_TCP_H_

#include <fvutils/net/fuse_server.h>
#include <fvutils/net/fuse.h>

#include <sys/socket.h>
#include <sys/types.h>
#include <netinet/in.h>
#include <queue>
#include <list>
#include <string>

class FuseServerTCP : public FuseServer {
 public:
  FuseServerTCP(unsigned short port, unsigned int timeout = 5);
  virtual ~FuseServerTCP();

  virtual void          bind();
  virtual bool          accept();
  virtual void          close();
  virtual bool          connected();
  /** Send the current buffer
   * NOTE: This can cause a SIGPIPE signal if the client teared down the connection
   * you need to handle this externally, for example using SignalManager
   * There is error handling in place if you just set the SignalManager to ignore
   * SIGPIPEs! Use connected() to detect if a connection has been teared down
   */
  virtual void          send();
  virtual void          disconnect(unsigned int client = 0xFFFFFFFF);
  virtual void          process();


  // Image related
  virtual void          setImageColorspace(unsigned int image_num, colorspace_t cspace);
  virtual void          setImageWidth(unsigned int image_num, unsigned int width);
  virtual void          setImageHeight(unsigned int image_num, unsigned int height);
  virtual void          setImageDimensions(unsigned int image_num,
					   unsigned int w, unsigned int h);
  virtual void          setImageROI(unsigned int image_num,
				    unsigned int x, unsigned int y,
				    unsigned int w, unsigned int h);
  virtual void          setImageCircle(unsigned int image_num,
				       int x, int y, unsigned int r);
  virtual void          setImageCircleFound(unsigned int image_num, bool found);
  virtual void          setImageBuffer(unsigned int image_num, unsigned char *buffer);
  virtual std::vector<unsigned int> getImageNumbers();
  virtual std::vector<unsigned int> getRequestedImageNumbers();
  virtual void          setImageAvailable(unsigned int image_num, bool available);
  virtual void          setImageCopyMode(FuseServer::FuseServer_copy_mode_t mode);
  virtual void          setImageAppAlive(unsigned int image_num,
					 bool alive,
					 std::string appl = "");

  // for LUT download (server to client)
  virtual void          setLUT(unsigned int lut_id,
			       unsigned int width, unsigned int height,
			       unsigned int bytes_per_cell,
			       unsigned char *data);
  virtual std::vector<unsigned int> getLutIDs();
  virtual std::vector<unsigned int> getRequestedLutIDs();
  virtual void          setLutAvailable(unsigned int lut_id, bool available);
  virtual void          setLutAppAlive(unsigned int lut_id,
				       bool alive,
				       std::string appl = "");

  // for LUT upload (client to server)
  virtual std::vector<unsigned int> getUploadedLutIDs();
  virtual unsigned char * getUploadedLutBuffer(unsigned int lut_id);
  virtual unsigned int    getUploadedLutBufferSize(unsigned int lut_id);
  virtual unsigned int    getUploadedLutWidth(unsigned int lut_id);
  virtual unsigned int    getUploadedLutHeight(unsigned int lut_id);
  virtual unsigned int    getUploadedLutBytesPerCell(unsigned int lut_id);
  virtual void            setUploadLutSuccess(unsigned int lut_id, bool success);
  virtual void            setUploadLutAppAlive(unsigned int lut_id,
					       bool alive,
					       std::string appl = "");

  // messaging stuff (client to server)
  virtual std::map< unsigned int, std::vector< std::pair< long, unsigned int > > >
    getMessageSubscriptions();
  virtual bool            isMessageAvailable();
  virtual void            getMessage(unsigned int *msgqid,
				     char **msg, unsigned int *msg_size);
  virtual void            freeMessage();
  virtual void            sendMessage(unsigned int msgqid, long mtype,
				      char *msg, unsigned int msg_size);
  virtual void            removeMessageQueueSubscriptions( unsigned int msgqid );


 private:
  bool          data_available(unsigned int client);
  void          recv(unsigned int client);

  unsigned int  recv_packet(unsigned int client, FUSE_packet_type_t packet_type,
			    unsigned char *buf, unsigned int buf_size);
  unsigned int  recv_data(unsigned int client, unsigned char *buf, unsigned int buf_size);
  void          send_packet(unsigned int client,
			    FUSE_packet_type_t packet_type,
			    unsigned char *buf,
			    unsigned int buf_size);
  unsigned int  send_data(unsigned int client, unsigned char *buf, unsigned int buf_size);

  void          close_image(unsigned int image_num);
  void          close_lut(unsigned int lut_id);
  void          open_image(unsigned int image_num);
  void          open_lut(unsigned int lut_id);

  void          check_and_resize_buffer(unsigned int image_num);

  std::string msg_prefix;
  unsigned short port;
  int sockfd;
  struct sockaddr_in serv_addr;
  unsigned int timeout;

  unsigned int next_client_num;

  // here first unsigned int means client id
  std::map< unsigned int, int >            client_sockfds;
  std::map< unsigned int, bool >           image_requested;
  std::map< unsigned int, bool >           lut_requested;
  std::map< unsigned int, bool >           image_info_requested;
  // maps a client id to an image id
  std::map< unsigned int, unsigned int >   image_request;
  // maps a client id to a lut id
  std::map< unsigned int, unsigned int >   lut_request;
  std::map< unsigned int, unsigned int >   lut_upload;

  // here first unsigned int means image id
  std::map< unsigned int, unsigned int >             img_buffer_sizes;
  std::map< unsigned int, unsigned char *>           img_buffers;
  std::map< unsigned int, FUSE_image_packet_header_t > image_headers;
  std::map< unsigned int, unsigned int >             image_refcount;
  std::map< unsigned int, bool >                     image_available;
  std::map< unsigned int, bool >                     image_needs_refresh;
  std::map< unsigned int, std::string >              image_app;
  std::map< unsigned int, bool >                     image_app_alive;

  // here first unsigned int means lut id
  std::map< unsigned int, FUSE_lookuptable_packet_header_t > lut_headers;
  std::map< unsigned int, unsigned int >             lut_buffer_sizes;
  std::map< unsigned int, unsigned char * >          lut_buffers;
  std::map< unsigned int, unsigned int >             lut_refcount;
  std::map< unsigned int, bool >                     lut_available;
  std::map< unsigned int, bool >                     lut_needs_refresh;
  std::map< unsigned int, std::string >              lut_app;
  std::map< unsigned int, bool >                     lut_app_alive;

  std::map< unsigned int, FUSE_lookuptable_packet_header_t > lut_upload_headers;
  std::map< unsigned int, unsigned int >             lut_upload_buffer_sizes;
  std::map< unsigned int, unsigned char * >          lut_upload_buffers;
  std::map< unsigned int, bool >                     lut_upload_send_success;
  std::map< unsigned int, std::string >              lut_upload_app;
  std::map< unsigned int, bool >                     lut_upload_app_alive;

  typedef struct {
    unsigned int client_id;
    long         mtype;
    unsigned int data_size;
  } msg_subscr_t;
  // here first unsigned int means message queue id
  // second is a vector of subscribed client ids
  std::map< unsigned int, std::list< msg_subscr_t > >   msg_subscr;

  // incoming messages
  std::queue< FUSE_message_t * >                          messages;

  FuseServer_copy_mode_t image_copy_mode;

  static const unsigned int IMAGE_ALL;
  static const unsigned int LUT_ALL;

};


#endif
