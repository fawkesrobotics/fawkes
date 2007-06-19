
/***************************************************************************
 *  fuse_client_tcp.cpp - network image transport client TCP implementation
 *
 *  Generated: Tue Jan 10 13:35:00 2006
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

#include <fvutils/net/fuse_client_tcp.h>
#include <utils/system/console_colors.h>

#include <sys/socket.h>
#include <sys/types.h>
#include <sys/time.h>
#include <netinet/in.h>
#include <netdb.h>
#include <fcntl.h>
#include <errno.h>
#include <cmath>

#include <iostream>

using namespace std;

/** @class FuseClientTCP <fvutils/net/fuse_client_tcp.h>
 * FUSE Client implementation using TCP.
 * This class provides a FUSE client that uses the TCP protocol to transfer the
 * data between client and server.
 * @author Tim Niemueller
 */

/** Constructor.
 * @param hostname hostname or IP address in textual representation as dot
 * separated octets to connect to
 * @param port TCP port to connect to
 * @param timeout connection timeout
 */
FuseClientTCP::FuseClientTCP(const char *hostname, unsigned short port,
			     float timeout)
{
  this->port     = port;
  this->hostname = hostname;

  network_timeout = fabs( timeout );

  msg_prefix = cblue + "FuseClientTCP: " + cnormal;

  is_connected = false;
  image_num       = 0;
  image_width     = 0;
  image_height    = 0;
  image_cspace    = CS_UNKNOWN;

  roi_x         = 0;
  roi_y         = 0;
  roi_width     = 0;
  roi_height    = 0;
  circle_x      = 0;
  circle_y      = 0;
  circle_radius = 0;
  circle_found  = false;

  buffer    = NULL;
  lut_buffer = NULL;

  buffer_size = 0;
  lut_buffer_size = 0;

  req_image = false;
  req_lut   = false;
  req_image_info = false;
  wait_image = false;
  wait_lut = false;

  lut_upload_buffer = NULL ;
  lut_upload_buffer_size = 0;
  lut_upload_lut_id = 0;
  lut_upload_width = 0;
  lut_upload_height = 0;
  lut_upload_bpc = 0;
  lut_upload_requested = false;
  lut_upload_success = false;

  last_result_extra = "";
}


/** Destructor. */
FuseClientTCP::~FuseClientTCP()
{
}


bool
FuseClientTCP::connect()
{

  if (is_connected) return true;

  struct sockaddr_in serv_addr;
  struct hostent *server;

  sockfd = socket(AF_INET, SOCK_STREAM, 0);
  if (sockfd < 0) {
    // error
    perror("Could not open socket");
    return false;
  }

  if (network_timeout > 0.f) {
    // set to non-blocking
    if ( fcntl(sockfd, F_SETFL, O_NONBLOCK) == -1 ) {
      perror("Could not set non-blocking");
      return false;
    }
  }

  server = gethostbyname( hostname );

  if ( ! server ) {
    return false;
  }
  bzero((char *) &serv_addr, sizeof(serv_addr));
  serv_addr.sin_family = AF_INET;
  bcopy((char *)server->h_addr, 
	(char *)&serv_addr.sin_addr.s_addr,
	server->h_length);
  serv_addr.sin_port = htons(port);

  if (network_timeout == 0.f) {
    if (::connect(sockfd,(struct sockaddr *)&serv_addr,sizeof(serv_addr)) < 0) {
      // error
      perror("Could not connect");
      ::close(sockfd);
      return false;
    }
  } else {
    struct timeval start, now;
    float diff_sec;
    bool conn_est = false;
    gettimeofday(&start, NULL);
    do {
      if (::connect(sockfd,(struct sockaddr *)&serv_addr,sizeof(serv_addr)) < 0) {
	if ( (errno != EINPROGRESS) &&
	     (errno != EALREADY) ) {
	  // error
	  perror("Could not connect");
	  ::close(sockfd);
	  return false;
	}
      } else {
	conn_est = true;
      }
      gettimeofday(&now, NULL);
      diff_sec = now.tv_sec - start.tv_sec + (now.tv_usec - start.tv_usec) / 100000;
    } while ( !conn_est && (diff_sec < network_timeout) );

    if (! conn_est) {
      cout << "Connection attempt timed out" << endl;
      ::close(sockfd);
      return false;
    }
  }

  FUSE_version_packet_t version_packet;
  version_packet.version     = htonl(1);

  send(FUSE_PT_VERSION, (unsigned char *)&version_packet, sizeof(version_packet));

  recv_packet(FUSE_PT_VERSION, (unsigned char *)&version_packet, sizeof(version_packet));

  if (ntohl(version_packet.version) == 1) {
    // valid version from server
    is_connected = true;

    return true;
  } else {
    // invalid version
    cout << "recieved invalid version" << endl;
    ::close(sockfd);
    return false;
  }

}


void
FuseClientTCP::disconnect()
{
  send(FUSE_PT_DISCONNECT, NULL, 0);

  ::close(sockfd);
  is_connected = false;
}


void
FuseClientTCP::failure_disconnect()
{
  ::close(sockfd);
  is_connected = false;
}


void
FuseClientTCP::recv()
{
  unsigned int res;

  if (! is_connected) {
    cout << "Trying to receive while not connected" << endl;
    return;
  }

  if (req_image_info) {
    FUSE_get_image_info_packet_t giip;
    giip.image_num = htonl(image_num);
    send( FUSE_PT_GET_IMAGE_INFO, (unsigned char *)&giip, sizeof(giip) );

    // for image info we block
    FUSE_image_info_packet_t iip;
    recv_packet(FUSE_PT_IMAGE_INFO, (unsigned char *)&iip, sizeof(iip) );
    
    check_and_change_image_info( fuse_ui2cs(ntohl(iip.colorspace)),
				 ntohl(iip.width),
				 ntohl(iip.height) );

    req_image_info = false;
  }

  if (req_image && ! wait_image) {
    FUSE_reqimage_packet_t imagereq;
    imagereq.image_num = htonl(image_num);
    send( FUSE_PT_REQIMAGE, (unsigned char *)&imagereq, sizeof(imagereq) );
    wait_image = true;
    req_image  = false;
    image_app_alive = true;
  }


  if (req_lut && ! wait_lut) {
    FUSE_getlut_packet_t getlut;
    getlut.lut_id = htonl(lut_id);
    send( FUSE_PT_GETLUT, (unsigned char *)&getlut, sizeof(getlut) );
    wait_lut = true;
    req_lut = false;
    lut_app_alive = true;
  }

  // recv
  while ( available() || wait_image || wait_lut ) {

    if ( ! available() ) {
      usleep( 5 );
      continue;
    }

    FUSE_header_t header;
    if (recv_data((unsigned char *)&header, sizeof(header)) != FUSE_RESULT_SUCCESS) {
      cout << msg_prefix << cred << "Invalid header received. Disconnecting." << cnormal << endl;
      disconnect();
      return;
    }

    unsigned int packet_type = ntohl(header.packet_type);
    // cout << msg_prefix << "Received packet of type " << packet_type << endl;

    switch (packet_type) {

    case FUSE_PT_RESULT:
      {
	FUSE_result_packet_t result;
	if (( res = recv_data( (unsigned char *)&result,
			       sizeof(result)))           == FUSE_RESULT_SUCCESS) {
	  unsigned int result_code = ntohl(result.result);
	  switch (result_code) {
	  case FUSE_RESULT_ERROR_IMAGE_UNAVAIL:
	    cout << msg_prefix << "Requested image not available" << endl;
	    wait_image = false;
	    break;

	  case FUSE_RESULT_ERROR_IMAGE_APPL_DEAD:
	    cout << msg_prefix << "Application for requested image is dead" << endl;
	    image_dead_app_name = result.extra;
	    wait_image = false;
	    image_app_alive = false;
	    break;

	  case FUSE_RESULT_ERROR_LUT_UNAVAIL:
	    cout << msg_prefix << "Requested LUT not available" << endl;
	    wait_lut = false;
	    break;

	  case FUSE_RESULT_ERROR_LUT_APPL_DEAD:
	    cout << msg_prefix << "Application for requested LUT is dead" << endl;
	    lut_dead_app_name = result.extra;
	    lut_app_alive = false;
	    wait_lut = false;
	    break;

	  default:
	    cout << msg_prefix << "Not specified result received: " << result_code << endl;
	    break;
	  }
	} else {
	  cout << msg_prefix << cred << "Could not read result packet" << cnormal << endl;
	}
      }
      break;

    case FUSE_PT_IMAGE:
      {
	FUSE_image_packet_header_t image_header;
	if (( res = recv_data( (unsigned char *)&image_header,
			       sizeof(image_header)))           != FUSE_RESULT_SUCCESS) {

	  cout << msg_prefix << "could not read image_header" << endl;
	  image_available = false;
	} else if ( ntohl(image_header.image_num) != image_num ) {
	  cout << msg_prefix << "Received wrong image, requested " << image_num
	       << " but got " << ntohl(image_header.image_num) << endl;
	  image_available = false;
	} else {

	  check_and_change_image_info( fuse_ui2cs(ntohl(image_header.colorspace)),
				       ntohl(image_header.width),
				       ntohl(image_header.height) );

	  roi_x         = ntohl(image_header.roi_x);
	  roi_y         = ntohl(image_header.roi_y);
	  roi_width     = ntohl(image_header.roi_width);
	  roi_height    = ntohl(image_header.roi_height);
	  circle_x      = ntohl(image_header.circle_x);
	  circle_y      = ntohl(image_header.circle_y);
	  circle_radius = ntohl(image_header.circle_radius);
	  circle_found  = (image_header.flag_circle_found == 1);

	  recv_data(buffer, buffer_size);
	  image_available = true;
	}
	wait_image = false;
      }
      break;

    case FUSE_PT_LUT:
      {
	FUSE_lookuptable_packet_header_t lut_header;
	if ( (res = recv_data((unsigned char *)&lut_header, sizeof(lut_header)))
	     != FUSE_RESULT_SUCCESS) {
	  lut_available = false;
	} else {

	  /*
	    cout << "received header  lut_id=" << ntohl(lut_header.lut_id)
	    << "  w=" << ntohl(lut_header.width)
	    << "  h=" << ntohl(lut_header.height) << endl;
	  */

	  if ( (ntohl(lut_header.width) != lut_width) ||
	       (ntohl(lut_header.height) != lut_height) ||
	       (ntohl(lut_header.bytes_per_cell) != lut_bpc) ) {

	    lut_width = ntohl(lut_header.width);
	    lut_height = ntohl(lut_header.height);
	    lut_bpc = ntohl(lut_header.bytes_per_cell);
	    lut_buffer_size = lut_width * lut_height * lut_bpc;
      
	    if (lut_buffer != NULL) {
	      free(lut_buffer);
	    }
	    lut_buffer = (unsigned char *)malloc(lut_buffer_size);
	  }

	  recv_data(lut_buffer, lut_buffer_size);
	  lut_available = true;
	}
	wait_lut = false;
      }
      break;

    case FUSE_PT_MESSAGE:
      {
	FUSE_message_packet_header_t mh;
	if ( recv_data((unsigned char*)&mh, sizeof(mh)) == FUSE_RESULT_SUCCESS) {
	  FUSE_message_t *m = (FUSE_message_t *)malloc(sizeof(FUSE_message_t));
	  //cout << "received message, msgqid=" << mh.msgqid
	  //     << "  data_size=" << mh.data_size << endl;
	  m->msgqid = mh.msgqid;
	  m->msg_size = mh.data_size;
	  m->msg = (char *)malloc(mh.data_size);
	  recv_data((unsigned char *)m->msg, mh.data_size);
	  messages.push(m);
	}
      }
      break;

    case FUSE_PT_DISCONNECT:
      {
	cout << msg_prefix << cyellow << "server disconnected gracefully" << cnormal << endl;
	disconnect();
      }

    default:
      cout << msg_prefix << cred << "Unknown packet type received: " << packet_type
	   << cnormal << endl;
      break;
    }

  }

}


void
FuseClientTCP::send()
{
  if (! is_connected) {
    cout << "Trying to send while not connected" << endl;
    return;
  }

  if (lut_upload_requested) {

    if ( (lut_upload_buffer != NULL) &&
	 (lut_upload_buffer_size > 0) &&
	 (lut_upload_width > 0) &&
	 (lut_upload_height > 0) &&
	 (lut_upload_bpc > 0) ) {

      FUSE_lookuptable_packet_header_t setlut;
      setlut.lut_id = htonl(lut_upload_lut_id);
      setlut.width  = htonl(lut_upload_width);
      setlut.height = htonl(lut_upload_height);
      setlut.bytes_per_cell = htonl(lut_upload_bpc);
      send( FUSE_PT_SETLUT, (unsigned char *)&setlut, sizeof(setlut) );

      send_data(lut_upload_buffer, lut_upload_buffer_size);

      FUSE_result_packet_t result;
      if (recv_packet(FUSE_PT_RESULT, (unsigned char*)&result, sizeof(result)) != FUSE_RESULT_SUCCESS) {
	lut_upload_success = false;
      } else {
	lut_upload_success = (ntohl(result.result) == FUSE_RESULT_SUCCESS);
      }
    } else {
      lut_upload_success = false;
    }

    lut_upload_requested = false;
  }
}

colorspace_t
FuseClientTCP::getImageColorspace() const
{
  return image_cspace;
}


unsigned int
FuseClientTCP::getImageWidth() const
{
  return image_width;
}


unsigned int
FuseClientTCP::getImageHeight() const
{
  return image_height;
}


void
FuseClientTCP::getImageDimensions(unsigned int *w, unsigned int *h) const
{
  *w = image_width;
  *h = image_height;
}


unsigned char *
FuseClientTCP::getImageBuffer()
{
  return buffer;
}


bool
FuseClientTCP::setImageNumber(unsigned int num)
{
  image_num = num;

  return true;
}


bool
FuseClientTCP::imageAvailable() const
{
  return image_available;
}


bool
FuseClientTCP::isImageApplAlive() const
{
  return image_app_alive;
}


string
FuseClientTCP::getImageDeadAppName() const
{
  return image_dead_app_name;
}


bool
FuseClientTCP::setLutID(unsigned int lut_id)
{
  this->lut_id = lut_id;
  return true;
}


unsigned int
FuseClientTCP::getLutWidth() const
{
  return lut_width;
}


unsigned int
FuseClientTCP::getLutHeight() const
{
  return lut_height;
}


unsigned int
FuseClientTCP::getLutBytesPerCell() const
{
  return lut_bpc;
}


unsigned char *
FuseClientTCP::getLutBuffer() const
{
  return lut_buffer;
}


unsigned int
FuseClientTCP::getLutBufferSize() const
{
  return lut_buffer_size;
}


bool
FuseClientTCP::isLutApplAlive() const
{
  return lut_app_alive;
}


string
FuseClientTCP::getLutDeadAppName() const
{
  return lut_dead_app_name;
}


void
FuseClientTCP::setLutUpload(unsigned int lut_id,
			    unsigned int width, unsigned int height,
			    unsigned int bytes_per_cell,
			    unsigned char *buffer, unsigned int buffer_size)
{
  lut_upload_lut_id      = lut_id;
  lut_upload_width       = width;
  lut_upload_height      = height;
  lut_upload_bpc         = bytes_per_cell;
  lut_upload_buffer      = buffer;
  lut_upload_buffer_size = buffer_size;
}


void
FuseClientTCP::requestLutUpload(bool request)
{
  lut_upload_requested = request;
  if (request) {
    lut_upload_success = false;
  }
}


bool
FuseClientTCP::lutUploadSuccess()
{
  return lut_upload_success;
}


bool
FuseClientTCP::available()
{
  if (! is_connected) return false;

  fd_set rfds;
  struct timeval tv;
  int retval = 1;

  FD_ZERO(&rfds);
  FD_SET(sockfd, &rfds);
  tv.tv_sec = 0;
  tv.tv_usec = 0;

  retval = select(sockfd + 1, &rfds, NULL, NULL, &tv);

  return (retval > 0);
}


bool
FuseClientTCP::connected()
{
  return is_connected;
}

void
FuseClientTCP::send(FUSE_packet_type_t packet_type,
		    unsigned char *buf, unsigned int buf_size)
{

  FUSE_header_t header;
  header.packet_type = htonl(packet_type);
  send_data((unsigned char *)&header, sizeof(header));

  send_data(buf, buf_size);
}


void
FuseClientTCP::send_data(unsigned char *buf, unsigned int buf_size)
{
  int retval = 0;
  unsigned int bytes_written = 0;

  while ((retval != -1) && (bytes_written < buf_size)) {
    retval = write(sockfd, buf + bytes_written, buf_size - bytes_written);
    if (retval < 0) {
      if (errno != EAGAIN) {
	failure_disconnect();
      } else {
	// just to meet loop condition
	retval = 0;
      }
    } else {
      bytes_written += retval;
    }
  }
}

unsigned int
FuseClientTCP::recv_packet(FUSE_packet_type_t packet_type,
			   unsigned char *buf,
			   unsigned int buf_size)
{

  FUSE_header_t header;
  if (recv_data( (unsigned char *)&header, sizeof(header) ) != FUSE_RESULT_SUCCESS) {
    return FUSE_RESULT_ERROR;
  }

  if ( ntohl(header.packet_type) != packet_type ) {
    if ( ntohl(header.packet_type) == FUSE_PT_RESULT ) {
      // if a FUSE_PT_RESULT packet should not be retrieved
      // but we actually got one, retrieve it and return the result
      // state, it should not be retrieved since otherwise
      // we would have packet_type == ntohl(header.packet_type) == FUSE_PT_RESULT
      // and we would not be in this case here

      FUSE_result_packet_t result;
      recv_data( (unsigned char *)&result, sizeof(result));
      // never return success if we did not receive what we expected

      last_result_extra = result.extra;

      if (ntohl(result.result) != FUSE_RESULT_SUCCESS) {
	return ntohl(result.result);
      } else {
	return FUSE_RESULT_ERROR;
      }

    } else {
      cout << "received unexpected packet, got " << ntohl(header.packet_type)
	   << " but expected " << packet_type << endl;

      failure_disconnect();
      return FUSE_RESULT_ERROR;
    }
  } else {
    recv_data(buf, buf_size);
    return FUSE_RESULT_SUCCESS;
  }
}


unsigned int
FuseClientTCP::recv_data(unsigned char *buf, unsigned int buf_size)
{
  int retval = 0;
  unsigned int bytes_read = 0;
  struct timeval start, now;
  float diff_sec;

  gettimeofday(&start, NULL);

  do {
    retval = read(sockfd, buf + bytes_read, buf_size - bytes_read);
    if (retval < 0) {
      if (errno != EAGAIN) {
	failure_disconnect();
	return FUSE_RESULT_ERROR;
      } else {
	// just to meet loop condition
	retval = 0;
      }
    } else {
      bytes_read += retval;
      // reset timeout
      gettimeofday(&start, NULL);
    }
    gettimeofday(&now, NULL);
    diff_sec = now.tv_sec - start.tv_sec + (now.tv_usec - start.tv_usec) / 100000;
    usleep(0);
  } while ((retval != -1) && (bytes_read < buf_size) && (diff_sec < network_timeout) );

  if ( (retval == -1) ||
       (bytes_read < buf_size) ||
       diff_sec >= network_timeout ) {
    return FUSE_RESULT_ERROR_TIMEOUT;
  }

  return FUSE_RESULT_SUCCESS;
}


void
FuseClientTCP::check_and_change_image_info(colorspace_t cspace,
					   unsigned int width, unsigned int height)
{

  if ( (cspace != image_cspace) ||
       (width  != image_width) ||
       (height != image_height) ) {
    image_cspace = cspace;
    image_width  = width;
    image_height = height;
    buffer_size  = colorspace_buffer_size(image_cspace, image_width, image_height);
      
    if (buffer != NULL) {
      free(buffer);
    }
    buffer=(unsigned char *)malloc(buffer_size);
  }
}


void
FuseClientTCP::requestImage(bool request)
{
  req_image = request;
}


void
FuseClientTCP::requestImageInfo(bool request)
{
  req_image_info = request;
}

void
FuseClientTCP::requestLut(bool request)
{
  req_lut = request;
}


unsigned int
FuseClientTCP::getImageNumber() const
{
  return image_num;
}


unsigned int
FuseClientTCP::getLutID() const
{
  return lut_id;
}


bool
FuseClientTCP::lutAvailable() const
{
  return lut_available;
}


unsigned int
FuseClientTCP::getRoiX() const
{
  return roi_x;
}


unsigned int
FuseClientTCP::getRoiY() const
{
  return roi_y;
}


unsigned int
FuseClientTCP::getRoiWidth() const
{
  return roi_width;
}


unsigned int
FuseClientTCP::getRoiHeight() const
{
  return roi_height;
}


int
FuseClientTCP::getCircleX() const
{
  return circle_x;
}


int
FuseClientTCP::getCircleY() const
{
  return circle_y;
}


unsigned int
FuseClientTCP::getCircleRadius() const
{
  return circle_radius;
}



bool
FuseClientTCP::getCircleFound() const
{
  return circle_found;
}


bool
FuseClientTCP::isMessageAvailable()
{
  return (messages.size() > 0);
}


void
FuseClientTCP::getMessage(unsigned int *msgqid,
			  char **msg, unsigned int *msg_size)
{
  if (messages.size() == 0) return;

  FUSE_message_t *t = messages.front();
  *msgqid           = t->msgqid;
  *msg              = t->msg;
  *msg_size         = t->msg_size;
}


void
FuseClientTCP::freeMessage()
{
  FUSE_message_t *t = messages.front();
  messages.pop();
  free(t->msg);
  free(t);
}


void
FuseClientTCP::dropMessage()
{
  // messages.front()->msg is NOT freed
  delete messages.front();
  messages.pop();
}


void
FuseClientTCP::sendMessage(unsigned int msgqid,
			   char *msg, unsigned int msg_size)
{

  FUSE_message_packet_header_t msg_header;
  msg_header.msgqid = msgqid;
  msg_header.data_size = msg_size;

  send(FUSE_PT_MESSAGE, (unsigned char*)&msg_header, sizeof(msg_header));
  send_data((unsigned char *)msg, msg_size);
}


void
FuseClientTCP::subscribeMessageQueue(unsigned int msgqid, long mtype,
				     unsigned int data_size)
{
  FUSE_message_subscribe_packet_t ms;
  ms.msgqid    = msgqid;
  ms.mtype     = mtype;
  ms.data_size = data_size;

  send(FUSE_PT_MESSAGE_SUBSCRIBE, (unsigned char*)&ms, sizeof(ms));
}


void
FuseClientTCP::unsubscribeMessageQueue(unsigned int msgqid, long mtype)
{
  FUSE_message_unsubscribe_packet_t ms;
  ms.msgqid = msgqid;
  ms.mtype  = mtype;

  send(FUSE_PT_MESSAGE_UNSUBSCRIBE, (unsigned char*)&ms, sizeof(ms));
}
