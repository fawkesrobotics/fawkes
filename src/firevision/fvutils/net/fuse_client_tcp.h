
/***************************************************************************
 *  fuse_client_tcp.h - network image transport client tcp implementation
 *
 *  Generated: Tue Jan 10 13:32:43 2006
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

#ifndef __FIREVISION_FVUTILS_NET_FUSE_CLIENT_TCP_H_
#define __FIREVISION_FVUTILS_NET_FUSE_CLIENT_TCP_H_

#include <fvutils/net/fuse_client.h>
#include <fvutils/net/fuse.h>
#include <queue>
#include <string>

class FuseClientTCP :FuseClient {
 public:

  FuseClientTCP(const char *hostname, unsigned short port, float timeout = 5.0);
  virtual ~FuseClientTCP();

  virtual bool             connect();
  virtual void             recv();
  virtual void             send();
  virtual void             disconnect();
  virtual bool             connected();

  virtual colorspace_t     getImageColorspace() const;
  virtual unsigned int     getImageWidth() const;
  virtual unsigned int     getImageHeight() const;
  virtual void             getImageDimensions(unsigned int *w, unsigned int *h) const;
  virtual unsigned char *  getImageBuffer();
  virtual bool             setImageNumber(unsigned int num);
  virtual void             requestImage(bool request = true);
  virtual unsigned int     getImageNumber() const;
  virtual bool             imageAvailable() const;
  virtual bool             isImageApplAlive() const;
  virtual std::string      getImageDeadAppName() const;
  virtual void             requestImageInfo(bool request = true);

  virtual unsigned int     getRoiX() const;
  virtual unsigned int     getRoiY() const;
  virtual unsigned int     getRoiWidth() const;
  virtual unsigned int     getRoiHeight() const;
  virtual int              getCircleX() const;
  virtual int              getCircleY() const;
  virtual unsigned int     getCircleRadius() const;
  virtual bool             getCircleFound() const;

  virtual bool             setLutID(unsigned int lut_id);
  virtual unsigned int     getLutWidth() const;
  virtual unsigned int     getLutHeight() const;
  virtual unsigned int     getLutBytesPerCell() const;
  virtual unsigned char *  getLutBuffer() const;
  virtual unsigned int     getLutBufferSize() const;
  virtual void             requestLut(bool request = true);
  virtual unsigned int     getLutID() const;
  virtual bool             lutAvailable() const;
  virtual bool             isLutApplAlive() const;
  virtual std::string      getLutDeadAppName() const;

  virtual void             setLutUpload(unsigned int lut_id,
					unsigned int width, unsigned int height,
					unsigned int bytes_per_cell,
					unsigned char *buffer, unsigned int buffer_size);
  virtual void             requestLutUpload(bool request = true);
  virtual bool             lutUploadSuccess();

  virtual void             subscribeMessageQueue(unsigned int msgqid, long mtype,
						 unsigned int data_size);
  virtual void             unsubscribeMessageQueue(unsigned int msgqid, long mtype);
  virtual bool             isMessageAvailable();
  virtual void             getMessage(unsigned int *msgqid,
				      char **msg, unsigned int *msg_size);
  virtual void             freeMessage();
  virtual void             dropMessage();
  virtual void             sendMessage(unsigned int msgqid,
				       char *msg, unsigned int msg_size);


 private:
  bool             available();
  void             send(FUSE_packet_type_t packet_type,
			unsigned char *buffer = NULL, unsigned int buffer_size = 0);
  void             send_data(unsigned char *buf, unsigned int buf_size);
  unsigned int     recv_packet( FUSE_packet_type_t packet_type,
				unsigned char *buf, unsigned int buf_size);
  unsigned int     recv_data(unsigned char *buf, unsigned int buf_size);

  void             check_and_change_image_info(colorspace_t cpsace,
					       unsigned int width, unsigned int height);
  void             failure_disconnect();

 private:
  std::string    msg_prefix;

  unsigned char *buffer;
  unsigned int   buffer_size;

  unsigned int   image_num;
  unsigned int   image_width;
  unsigned int   image_height;
  colorspace_t   image_cspace;

  unsigned int   roi_x;
  unsigned int   roi_y;
  unsigned int   roi_width;
  unsigned int   roi_height;
  int            circle_x;
  int            circle_y;
  unsigned int   circle_radius;
  bool           circle_found;

  unsigned int   lut_id;
  unsigned int   lut_width;
  unsigned int   lut_height;
  unsigned int   lut_bpc;
  unsigned char *lut_buffer;
  unsigned int   lut_buffer_size;

  unsigned char *lut_upload_buffer;
  unsigned int   lut_upload_buffer_size;
  unsigned int   lut_upload_lut_id;
  unsigned int   lut_upload_width;
  unsigned int   lut_upload_height;
  unsigned int   lut_upload_bpc;
  bool           lut_upload_requested;
  bool           lut_upload_success;

  // incoming messages
  std::queue< FUSE_message_t * >  messages;

  const char *hostname;
  unsigned short port;

  std::string  last_result_extra;
  std::string  image_dead_app_name;
  std::string  lut_dead_app_name;

  int sockfd;
  bool is_connected;

  bool image_available;
  bool lut_available;

  bool image_app_alive;
  bool lut_app_alive;

  bool req_image;
  bool req_lut;
  bool req_image_info;

  bool wait_image;
  bool wait_lut;

  float network_timeout;
};


#endif
