
/***************************************************************************
 *  fuse_server_tcp.cpp - network image transport server TCP implementation
 *
 *  Generated: Tue Jan 10 11:09:54 2006
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

#include <core/exception.h>

#include <fvutils/net/fuse_server_tcp.h>
#include <fvutils/net/fuse.h>
#include <utils/system/console_colors.h>

#include <iostream>
#include <unistd.h>
#include <errno.h>
#include <fcntl.h>
#include <sys/time.h>

using namespace std;

/** @class FuseServerTCP <fvutils/net/fuse_server_tcp.h>
 * FUSE server implementation using TCP.
 * This class provides a FUSE server using TCP to transmit data
 * to and from the client.
 * @author Tim Niemueller
 */

/** Image number constant for all images. */
const unsigned int FuseServerTCP::IMAGE_ALL    = 0xFFFFFFFF;
/** LUT ID constant for all LUTs. */
const unsigned int FuseServerTCP::LUT_ALL = 0xFFFFFFFF;

/** Constructor.
 * @param port TCP port to listen on
 * @param timeout connection timeout
 */
FuseServerTCP::FuseServerTCP(unsigned short port, unsigned int timeout)
{
  msg_prefix = cblue + "FuseServerTCP: " + cnormal;
  this->port = port;
  this->timeout = timeout;

  client_sockfds.clear();
  image_requested.clear();
  image_info_requested.clear();
  image_request.clear();
  image_refcount.clear();
  img_buffer_sizes.clear();
  img_buffers.clear();
  image_headers.clear();
  image_available.clear();
  image_needs_refresh.clear();
  image_app.clear();
  image_app_alive.clear();

  lut_requested.clear();
  lut_request.clear();
  lut_buffer_sizes.clear();
  lut_buffers.clear();
  lut_headers.clear();
  lut_available.clear();
  lut_needs_refresh.clear();
  lut_app.clear();
  lut_app_alive.clear();

  lut_upload.clear();
  lut_upload_headers.clear();
  lut_upload_buffer_sizes.clear();
  lut_upload_buffers.clear();
  lut_upload_send_success.clear();
  lut_upload_app.clear();
  lut_upload_app_alive.clear();

  next_client_num = 0;
}


/** Destructor. */
FuseServerTCP::~FuseServerTCP()
{
}


void
FuseServerTCP::bind()
{
  sockfd = socket(AF_INET, SOCK_STREAM, 0);
  if (sockfd < 0) {
    perror("Error creating socket");
  }

  // set to non-blocking
  if ( fcntl(sockfd, F_SETFL, O_NONBLOCK) == -1 ) {
    perror("Could not set non-blocking");
    throw string("Could not set non-blocking");
  }
  

  serv_addr.sin_family = AF_INET;
  serv_addr.sin_addr.s_addr = INADDR_ANY;
  serv_addr.sin_port = htons(port);

  if (::bind(sockfd, (struct sockaddr *) &serv_addr, sizeof(serv_addr)) < 0) {
    // error
    perror("Could not bind");
    throw string("Could not bind to port");
  }
  ::listen(sockfd, 1);
}


bool
FuseServerTCP::accept()
{
  sockaddr_in client_addr;
  socklen_t clilen = sizeof(client_addr);
  int client_sockfd = ::accept(sockfd, (struct sockaddr *) &client_addr, &clilen);
  if (client_sockfd < 0) {
    // error
    if ( errno != EWOULDBLOCK) {
      // We expect this to happen quite often so do not give an error message in this case
      perror("Could not accept");
    }
    return false;
  }

  client_sockfds[next_client_num]  = client_sockfd;
  image_requested[next_client_num] = false;
  image_info_requested[next_client_num] = false;

  FUSE_version_packet_t version_packet;
  if ( (recv_packet(next_client_num, FUSE_PT_VERSION,
		    (unsigned char *)&version_packet,
		    sizeof(FUSE_version_packet_t)) == FUSE_PT_INVALID) ||
       (ntohl(version_packet.version) != 1) ) {

    ::close(client_sockfd);
    client_sockfds.erase( next_client_num );
    image_requested.erase( next_client_num );
    image_info_requested.erase( next_client_num );
    return false;
  }

  send_packet(next_client_num, FUSE_PT_VERSION,
	      (unsigned char *)&version_packet, sizeof(version_packet));

  ++next_client_num;
  return true;
}


void
FuseServerTCP::send()
{
  std::map< unsigned int, int >::iterator it;

  for (it = client_sockfds.begin(); it != client_sockfds.end(); ++it) {

    if (lut_upload.find( (*it).first ) != lut_upload.end()) {
      // this client has a lut upload running
      FUSE_result_packet_t result;
      if (lut_upload_send_success[lut_upload[(*it).first]]) {
	// ack setting the lut
	result.result = htonl(FUSE_RESULT_SUCCESS);
      } else {
	// the request was not fulfilled in last cycle, error!
	result.result = htonl(FUSE_RESULT_ERROR);
      }
      send_packet((*it).first, FUSE_PT_RESULT, (unsigned char*)&result, sizeof(result));

      // erase all the data related to this request
      lut_upload_headers.erase( lut_upload[(*it).first] );
      lut_upload_buffer_sizes.erase( lut_upload[(*it).first] );
      free(lut_upload_buffers[lut_upload[(*it).first]]);
      lut_upload_buffers.erase( lut_upload[(*it).first] );
      lut_upload_send_success.erase( lut_upload[(*it).first] );
      lut_upload.erase( (*it).first );
      lut_upload_app_alive.erase( (*it).first );
    }

    if (image_requested[(*it).first]) {

      /*
	cout << "sending out image " << image_request[(*it).first] << " to client " << (*it).first
	<< "( need to send " << img_buffer_sizes[image_request[(*it).first]] << " bytes)" << endl
	<< "i=" << ntohl(image_headers[image_request[(*it).first]].image_num)
	<< "  w=" << ntohl(image_headers[image_request[(*it).first]].width)
	<< "  h=" << ntohl(image_headers[image_request[(*it).first]].height)
	<< "  c=" << ntohl(image_headers[image_request[(*it).first]].colorspace)
	<< endl;
      */
      //cout << image_request[(*it).first] << flush;

      
      if ( (img_buffers[image_request[(*it).first]] == NULL) ||
	   ! image_available[image_request[(*it).first]] ||
	   ! image_app_alive[image_request[(*it).first]] ) {

	cout << msg_prefix << cyellow
	     << "Could not send image: " << flush;

	FUSE_result_packet_t result;
	memset(&result, 0, sizeof(result));
	if (! image_app_alive[image_request[(*it).first]]) {
	  cout << "image app not alive";
	  result.result = htonl(FUSE_RESULT_ERROR_IMAGE_APPL_DEAD);
	  strncpy(result.extra, image_app[image_request[(*it).first]].c_str(), sizeof(result.extra)-1);
	} else if (! image_available[image_request[(*it).first]]) {
	  cout << "image not available";
	  result.result = htonl(FUSE_RESULT_ERROR_IMAGE_UNAVAIL);
	} else if (img_buffers[image_request[(*it).first]] == NULL) {
	  cout << "invalid buffer (NULL)";
	}

	cout << cnormal << endl;

	send_packet((*it).first, FUSE_PT_RESULT,
		    (unsigned char*)&result, sizeof(result));

      } else {

	send_packet((*it).first, FUSE_PT_IMAGE,
		    (unsigned char*)&image_headers[image_request[(*it).first]],
		    sizeof(FUSE_image_packet_header_t));

	send_data((*it).first, img_buffers[image_request[(*it).first]],
		  img_buffer_sizes[image_request[(*it).first]] );
	 
      }

      // Request fulfilled
      image_needs_refresh[image_request[(*it).first]] = false;
      image_requested[(*it).first] = false;
    }


    // Image info
    if (image_info_requested[(*it).first]) {

      FUSE_image_info_packet_t iip;
      iip.image_num  = image_headers[image_request[(*it).first]].image_num;
      iip.colorspace = image_headers[image_request[(*it).first]].colorspace;
      iip.width      = image_headers[image_request[(*it).first]].width;
      iip.height     = image_headers[image_request[(*it).first]].height;

      send_packet((*it).first, FUSE_PT_IMAGE_INFO,
		  (unsigned char*)&iip, sizeof(iip));
	 
      // Request fulfilled
      image_needs_refresh[image_request[(*it).first]] = false;
      image_info_requested[(*it).first] = false;
    }


    // LUT
    if (lut_requested[(*it).first]) {

      if ( (lut_buffers[lut_request[(*it).first]] == NULL) ||
	   ! lut_available[lut_request[(*it).first]] ||
	   ! lut_app_alive[lut_request[(*it).first]] ) {

	cout << msg_prefix << cyellow
	     << "Could not send lut: " << flush;

	FUSE_result_packet_t result;
	memset(&result, 0, sizeof(result));
	if (! lut_app_alive[lut_request[(*it).first]]) {
	  cout << "lut app (" << lut_app[lut_request[(*it).first]] << ") not alive";
	  result.result = htonl(FUSE_RESULT_ERROR_LUT_APPL_DEAD);
	  strncpy(result.extra, lut_app[lut_request[(*it).first]].c_str(), sizeof(result.extra)-1);
	} else if (! lut_available[lut_request[(*it).first]]) {
	  cout << "image not available";
	  result.result = htonl(FUSE_RESULT_ERROR_LUT_UNAVAIL);
	} else if (lut_buffers[lut_request[(*it).first]] == NULL) {
	  cout << "invalid buffer (NULL)";
	}
	cout << cnormal << endl;

	send_packet((*it).first, FUSE_PT_RESULT,
		    (unsigned char*)&result, sizeof(result));

      } else {
	send_packet((*it).first, FUSE_PT_LUT,
		    (unsigned char*)&lut_headers[lut_request[(*it).first]],
		    sizeof(FUSE_lookuptable_packet_header_t));	

	send_data((*it).first, lut_buffers[lut_request[(*it).first]],
		  lut_buffer_sizes[lut_request[(*it).first]]);

      }
      // Request fulfilled
      lut_needs_refresh[lut_request[(*it).first]] = false;
      lut_requested[(*it).first] = false;
    }
  }
}


void
FuseServerTCP::send_packet(unsigned int client,
			   FUSE_packet_type_t packet_type,
			   unsigned char *buf,
			   unsigned int buf_size)
{
  FUSE_header_t header;
  header.packet_type = htonl(packet_type);
  if (send_data(client, (unsigned char*)&header, sizeof(header)) == FUSE_RESULT_SUCCESS) {
    send_data(client, buf, buf_size);
  }
}


unsigned int
FuseServerTCP::send_data(unsigned int client, unsigned char *buf, unsigned int buf_size)
{
  unsigned int rv = FUSE_RESULT_SUCCESS;
  int retval = 0;
  unsigned int bytes_written = 0;

  timeval start, now;
  gettimeofday(&start, NULL);
  gettimeofday(&now, NULL);

  while (((unsigned int)(now.tv_sec - start.tv_sec) < timeout) &&
	 (retval != -1) &&
	 (bytes_written < buf_size)) {
    retval = write(client_sockfds[client], buf + bytes_written, buf_size - bytes_written);
    if (retval < 0) {
      if (errno != EAGAIN) {
	cout << msg_prefix << flush;
	perror("Error sending data");
	disconnect(client);
	rv = FUSE_RESULT_ERROR;
      } else {
	// just to meet loop condition
	retval = 0;
      }
    } else {
      bytes_written += retval;
    }
    gettimeofday(&now, NULL);
  }
  if ( (retval == -1) || ((unsigned int)(now.tv_sec - start.tv_sec) > timeout)) {
    // Either error or timeout
    rv = FUSE_RESULT_ERROR;
    cout << msg_prefix << "Write timed out" << endl;
    disconnect(client);
  }

  return rv;
}


unsigned int
FuseServerTCP::recv_packet(unsigned int client,
			   FUSE_packet_type_t packet_type,
			   unsigned char *buf,
			   unsigned int buf_size)
{  
  unsigned int rd_rv;

  FUSE_header_t header;
  if ((rd_rv = recv_data(client, (unsigned char *)&header, sizeof(header))) != FUSE_RESULT_SUCCESS) {
    return rd_rv;
  } else {
    if ( ntohl(header.packet_type) != packet_type ) {
      return FUSE_RESULT_ERROR;
    } else {
      return recv_data(client, buf, buf_size);
    }
  }
}


unsigned int
FuseServerTCP::recv_data(unsigned int client, unsigned char *buf, unsigned int buf_size)
{
  unsigned int rv = FUSE_RESULT_SUCCESS;
  int retval = 0;
  unsigned int bytes_read = 0;

  timeval start, now;
  gettimeofday(&start, NULL);
  gettimeofday(&now, NULL);

  while (((unsigned int)(now.tv_sec - start.tv_sec) < timeout) &&
	 (retval != -1) &&
	 (bytes_read < buf_size)) {
    retval = read(client_sockfds[client], buf + bytes_read, buf_size - bytes_read);
    if (retval < 0) {
      if (errno != EAGAIN) {
	cout << msg_prefix << flush;
	perror("Error reading data");
	disconnect(client);
	rv = FUSE_RESULT_ERROR;
      } else {
	// just to meet loop condition
	retval = 0;
      }
    } else {
      bytes_read += retval;
    }
    gettimeofday(&now, NULL);
    usleep(0);
    //cout << "time sec=" << (unsigned int)(now.tv_sec - start.tv_sec) << endl;
  }
  if ( (retval == -1) || ((unsigned int)(now.tv_sec - start.tv_sec) >= timeout)) {
    // Either error or timeout
    cout << msg_prefix << "Timed out" << endl;
    disconnect(client);
    rv = FUSE_RESULT_ERROR;
  }

  return rv;
}


void
FuseServerTCP::disconnect(unsigned int client)
{

  if (client == 0xFFFFFFFF) {
    // disconnect all clients
    std::map< unsigned int, int >::iterator it;

    for (it = client_sockfds.begin(); it != client_sockfds.end(); ++it) {
      ::close(client_sockfds[(*it).second]);
      close_image( image_requested[(*it).first] );
      close_lut( lut_requested[(*it).first] );
    }
    client_sockfds.clear();
    image_requested.clear();
    image_request.clear();
    lut_requested.clear();
    lut_request.clear();
    msg_subscr.clear();

    close_image( IMAGE_ALL );
    close_lut( LUT_ALL );

  } else if ( client_sockfds.find( client ) != client_sockfds.end() ) {
    cout << msg_prefix << "Disconnecting client " << client << endl;
    ::close(client_sockfds[client]);
    close_image( image_requested[client] );
    close_lut( lut_requested[client] );

    map<unsigned int, list< msg_subscr_t > >::iterator it;
    list< msg_subscr_t >::iterator  sit;
    for (it = msg_subscr.begin(); it != msg_subscr.end(); ++it) {
      sit = (*it).second.begin();
      while (sit != (*it).second.end()) {
	if ( (*sit).client_id == client ) {
	  sit = (*it).second.erase( sit );
	} else {
	  ++sit;
	}
      }
      if ((*it).second.empty()) {
	msg_subscr.erase( it );
      }
    }

    client_sockfds.erase( client );
    image_requested.erase( client );
    lut_requested.erase(client);
    image_info_requested.erase( client );
    image_request.erase( client );
    lut_request.erase( client );
  }
}


void
FuseServerTCP::close()
{
  disconnect();
  ::close(sockfd);
}


bool
FuseServerTCP::connected()
{
  return (client_sockfds.size() > 0);
}


void
FuseServerTCP::setImageCopyMode(FuseServer::FuseServer_copy_mode_t mode)
{
  image_copy_mode = mode;
}


void
FuseServerTCP::process()
{
  if (! connected()) return;

  std::map< unsigned int, int >::iterator it;

  for (it = client_sockfds.begin(); it != client_sockfds.end(); ++it) {
    while ( data_available((*it).first) ) {
      recv((*it).first);
    }
  }
}


bool
FuseServerTCP::data_available(unsigned int client)
{
  if ( client_sockfds.find(client) == client_sockfds.end() ) return false;

  fd_set rfds;
  struct timeval tv;
  int retval = 1;

  FD_ZERO(&rfds);
  FD_SET(client_sockfds[client], &rfds);
  tv.tv_sec = 0;
  tv.tv_usec = 0;

  retval = select(client_sockfds[client] + 1, &rfds, NULL, NULL, &tv);

  return (retval > 0);
}


void
FuseServerTCP::recv(unsigned int client)
{
  if ( client_sockfds.find(client) == client_sockfds.end() ) return;

  FUSE_header_t header;
  if (recv_data(client, (unsigned char *)&header, sizeof(header)) != FUSE_RESULT_SUCCESS) {
    return;
  }

  unsigned int packet_type = ntohl(header.packet_type);
  // cout << msg_prefix << "Received packet of type " << packet_type << endl;

  switch (packet_type) {

  case FUSE_PT_REQIMAGE:
    {
      FUSE_reqimage_packet_t reqimage;
      if ( recv_data(client, (unsigned char*)&reqimage, sizeof(reqimage)) == FUSE_RESULT_SUCCESS) {
	unsigned int image_num = ntohl(reqimage.image_num);
	
	if ( (image_request.find(client) == image_request.end()) ||
	     (image_num != image_request[client]) ) {
	  // the client did not request an image before or it requested
	  // a different image

	  open_image( image_num );

	  if (image_num != image_request[client]) {
	    // image changed close old image
	    close_image( image_request[client] );
	  }

	  image_request[client] = image_num;
	} // else requests the same image as before

	image_needs_refresh[image_num] = true;
	image_requested[client] = true;
      } else {
	cout << msg_prefix << "Could not read reqimage packet" << endl;
	disconnect(client);
      }
    }
    break;

  case FUSE_PT_GET_IMAGE_INFO:
    {
      FUSE_get_image_info_packet_t giip;
      if (recv_data(client, (unsigned char*)&giip, sizeof(giip)) == FUSE_RESULT_SUCCESS) {
	unsigned int image_num = ntohl(giip.image_num);
	
	if ( (image_request.find(client) == image_request.end()) ||
	     (image_num != image_request[client]) ) {
	  // the client did not request an image before or it requested
	  // a different image

	  open_image( image_num );

	  if (image_num != image_request[client]) {
	    // image changed close old image
	    close_image( image_request[client] );
	  }

	  image_request[client] = image_num;
	} // else requests the same image as before

	image_needs_refresh[image_num] = true;
	image_info_requested[client] = true;
      } else {
	cout << msg_prefix << "Could not read get_image_info packet" << endl;
	disconnect(client);
      }
    }
    break;

  case FUSE_PT_GETLUT:
    {
      FUSE_getlut_packet_t getlut;
      if ( recv_data(client, (unsigned char*)&getlut, sizeof(getlut)) == FUSE_RESULT_SUCCESS) {
	unsigned int lut_id = ntohl(getlut.lut_id);

	// cout << "got request for lut " << lut_id << endl;

	if ( (lut_request.find(client) == lut_request.end()) ||
	     (lut_id != lut_request[client]) ) {
	  // the client did not request a lut before or it requested
	  // a different lut now

	  open_lut( lut_id );

	  if (lut_id != lut_request[client]) {
	    // lut changed, close old lut
	    close_lut( lut_request[client] );
	  }

	  lut_request[client] = lut_id;
	} // else requests the same lut as before

	lut_needs_refresh[lut_id] = true;
	lut_requested[client] = true;
      } else {
	cout << msg_prefix << "Could not read getlut packet" << endl;
	disconnect(client);
      }
    }
    break;

  case FUSE_PT_SETLUT:
    {
      FUSE_lookuptable_packet_header_t setlut;
      if ( recv_data(client, (unsigned char*)&setlut, sizeof(setlut)) == FUSE_RESULT_SUCCESS) {
	unsigned int lut_id     = ntohl(setlut.lut_id);
	unsigned int lut_width  = ntohl(setlut.width);
	unsigned int lut_height = ntohl(setlut.height);
	unsigned int lut_bpc    = ntohl(setlut.bytes_per_cell);
	unsigned int lut_size   = lut_width * lut_height * lut_bpc;
	unsigned char *lut_buffer = (unsigned char *)malloc(lut_size);
	
	// fetch the data in any case, we must because the protocol requires this for now
	if (recv_data(client, lut_buffer, lut_size) == FUSE_RESULT_SUCCESS ) {

	  if ( lut_upload_send_success.find(lut_id) != lut_upload_send_success.end() ) {
	    // there is already a process uploading this very lut, send an error to
	    // all following clients

	    FUSE_result_packet_t result;
	    result.result = htonl(FUSE_RESULT_ERROR_ALREADY);
	    send_packet(client, FUSE_PT_RESULT, (unsigned char*)&result, sizeof(result));

	    free(lut_buffer);

	  } else {

	    // this request is ok, take it!
	    lut_upload_send_success[lut_id] = false;
	    lut_upload[client] = lut_id;

	    lut_upload_headers[lut_id] = setlut;
	    lut_upload_buffer_sizes[lut_id] = lut_size;
	    lut_upload_buffers[lut_id] = lut_buffer;
	    lut_upload_app_alive[lut_id] = false;
	    lut_upload_app[lut_id] = "";

	  }
	} else {
	  free(lut_buffer);
	}
      }
    }
    break;

  case FUSE_PT_MESSAGE_SUBSCRIBE:
    {
      FUSE_message_subscribe_packet_t ms;
      if ( recv_data(client, (unsigned char*)&ms, sizeof(ms)) == FUSE_RESULT_SUCCESS) {
	if ( msg_subscr.find( ms.msgqid ) == msg_subscr.end() ) {
	  list< msg_subscr_t > t;
	  msg_subscr[ms.msgqid] = t;
	}

	list< msg_subscr_t >::iterator res;
	for (res = msg_subscr[ms.msgqid].begin(); res !=  msg_subscr[ms.msgqid].end(); ++res) {
	  if ( ((*res).client_id == client) &&
	       ((*res).mtype == ms.mtype) ) break;
	}
	if ( res == msg_subscr[ms.msgqid].end() ) {
	  // not yet subscribed to this queue
	  msg_subscr_t msub;
	  msub.client_id = client;
	  msub.mtype     = ms.mtype;
	  msub.data_size = ms.data_size;
	  msg_subscr[ms.msgqid].push_back( msub );
	}
      }
    }
    break;

  case FUSE_PT_MESSAGE_UNSUBSCRIBE:
    {
      FUSE_message_unsubscribe_packet_t mu;
      if ( recv_data(client, (unsigned char*)&mu, sizeof(mu)) == FUSE_RESULT_SUCCESS) {
	if ( msg_subscr.find( mu.msgqid ) != msg_subscr.end() ) {
	  // there are actually subscriptions for this msgqid

	  list< msg_subscr_t >::iterator res;
	  for (res = msg_subscr[mu.msgqid].begin(); res !=  msg_subscr[mu.msgqid].end(); ++res) {
	    if ( ((*res).client_id == client) &&
		 ((*res).mtype == mu.mtype) ) break;
	  }
	  if ( res != msg_subscr[mu.msgqid].end() ) {
	    // this client really is subscribed to this queue
	    msg_subscr[mu.msgqid].erase(res);
	  }
	}
      }
    }
    break;

  case FUSE_PT_MESSAGE:
    {
      FUSE_message_packet_header_t mh;
      if ( recv_data(client, (unsigned char*)&mh, sizeof(mh)) == FUSE_RESULT_SUCCESS) {
	FUSE_message_t *m = (FUSE_message_t *)malloc(sizeof(FUSE_message_t));
	//cout << "received message, msgqid=" << mh.msgqid
	//     << "  data_size=" << mh.data_size << endl;
	m->msgqid = mh.msgqid;
	m->msg_size = mh.data_size;
	m->msg = (char *)malloc(mh.data_size);
	recv_data(client, (unsigned char *)m->msg, mh.data_size);
	messages.push(m);
      }
    }
    break;


  case FUSE_PT_DISCONNECT:
    {
      cout << msg_prefix << "client graceful disconnect" << endl;
      disconnect(client);
    }
    break;

  default:
    cout << msg_prefix << cred
	 << "Unknown packet type received: " << packet_type << cnormal << endl;
    disconnect(client);
    break;
  }
    
}

void
FuseServerTCP::check_and_resize_buffer(unsigned int image_num)
{
  if (img_buffer_sizes[image_num] !=
      colorspace_buffer_size( fuse_ui2cs(ntohl(image_headers[image_num].colorspace)),
			      ntohl(image_headers[image_num].width),
			      ntohl(image_headers[image_num].height)) ) {
    // need to resize buffer

    img_buffer_sizes[image_num] = colorspace_buffer_size( fuse_ui2cs(ntohl(image_headers[image_num].colorspace)),
							  ntohl(image_headers[image_num].width),
							  ntohl(image_headers[image_num].height));

    if (image_copy_mode == FuseServer::DEEP_COPY) {
      if (img_buffers[image_num] != NULL) {
	free( img_buffers[image_num] );
      }
      if (img_buffer_sizes[image_num] > 0) {
	img_buffers[image_num] = (unsigned char *)malloc(img_buffer_sizes[image_num]);
	if (img_buffers[image_num] == NULL ) {
	  cout << "Could not allocate memory" << endl;
	}
      } else {
	cout << "buffer size is 0" << endl;
	img_buffers[image_num] = NULL;
      }
    }
  }
}

void
FuseServerTCP::setImageColorspace(unsigned int image_num, colorspace_t cspace)
{
  if ( image_headers.find(image_num) == image_headers.end() ) {
    cout << "Could not setColorspace" << endl;
    return;
  }

  image_headers[image_num].colorspace = htonl(fuse_cs2ui(cspace));
  check_and_resize_buffer(image_num);
}


void
FuseServerTCP::setImageWidth(unsigned int image_num, unsigned int width)
{
  if ( image_headers.find(image_num) == image_headers.end() ) return;

  image_headers[image_num].width = htonl(width);
  check_and_resize_buffer(image_num);
}


void
FuseServerTCP::setImageHeight(unsigned int image_num, unsigned int height)
{
  if ( image_headers.find(image_num) == image_headers.end() ) {
    cout << "could not set height, image num " << image_num << " is invalid" << endl;
    return;
  }

  image_headers[image_num].height = htonl(height);
  check_and_resize_buffer(image_num);
}


void
FuseServerTCP::setImageDimensions(unsigned int image_num,
				  unsigned int w, unsigned int h)
{
  if ( image_headers.find(image_num) == image_headers.end() ) {
    cout << "could not set height, image num " << image_num << " is invalid" << endl;
    return;
  }

  image_headers[image_num].width  = htonl(w);
  image_headers[image_num].height = htonl(h);
  check_and_resize_buffer(image_num);
}


vector<unsigned int>
FuseServerTCP::getImageNumbers()
{
  vector< unsigned int > rv;
  rv.clear();

  std::map< unsigned int, unsigned int >::iterator it;

  for (it = image_refcount.begin(); it != image_refcount.end(); ++it) {
    rv.push_back( (*it).first );
  }

  return rv;
}


vector<unsigned int>
FuseServerTCP::getRequestedImageNumbers()
{
  vector< unsigned int > rv;
  rv.clear();

  std::map< unsigned int, unsigned int >::iterator it;

  for (it = image_refcount.begin(); it != image_refcount.end(); ++it) {
    if ( image_needs_refresh[(*it).first]) {
      rv.push_back( (*it).first );
    }
  }

  return rv;
}


void
FuseServerTCP::setImageBuffer(unsigned int image_num, unsigned char *buf)
{

  if ( img_buffers.find(image_num) == img_buffers.end() ) {
    cout << "cannot set buffer" << endl;
    return;
  }
  
  if (image_copy_mode == FuseServer::DEEP_COPY) {
    if (img_buffers[image_num] == NULL) {
      cout << "Cannot set image buffer, no buffer assigned. Set width, height and colorspace" << endl;
    } else {
      memcpy( img_buffers[image_num], buf, img_buffer_sizes[image_num]);
    }
  } else {
    img_buffers[image_num] = buf;
  }
}


void
FuseServerTCP::setImageROI(unsigned int image_num,
			   unsigned int x, unsigned int y, unsigned int w, unsigned int h)
{
  if ( image_headers.find(image_num) == image_headers.end() ) return;

  image_headers[image_num].roi_x       = htonl(x);
  image_headers[image_num].roi_y       = htonl(y);
  image_headers[image_num].roi_width   = htonl(w);
  image_headers[image_num].roi_height  = htonl(h);
}


void
FuseServerTCP::setImageCircle(unsigned int image_num, int x, int y, unsigned int r)
{
  if ( image_headers.find(image_num) == image_headers.end() ) return;

  image_headers[image_num].circle_x       = htonl(x);
  image_headers[image_num].circle_y       = htonl(y);
  image_headers[image_num].circle_radius  = htonl(r);
}


void
FuseServerTCP::setImageCircleFound(unsigned int image_num, bool found)
{
  if ( image_headers.find(image_num) == image_headers.end() ) return;

  image_headers[image_num].flag_circle_found = found ? 1 : 0;
}


void
FuseServerTCP::setImageAvailable(unsigned int image_num, bool available)
{
  if ( image_available.find(image_num) == image_available.end() ) return;

  image_available[image_num] = available;
}


void
FuseServerTCP::setImageAppAlive(unsigned int image_num,
				bool alive,
				std::string appl)
{
  if (image_app.find( image_num ) == image_app.end()) return;
  if (image_app_alive.find( image_num ) == image_app_alive.end()) return;

  image_app[image_num] = appl;
  image_app_alive[image_num] = alive;
}


void
FuseServerTCP::setLUT(unsigned int lut_id,
		      unsigned int width, unsigned int height,
		      unsigned int bytes_per_cell,
		      unsigned char *lut)
{

  if ( lut_headers.find(lut_id) == lut_headers.end() ) return;
  if ( lut_buffers.find(lut_id) == lut_buffers.end() ) return;

  lut_headers[lut_id].width  = htonl(width);
  lut_headers[lut_id].height = htonl(height);
  lut_headers[lut_id].bytes_per_cell = htonl(bytes_per_cell);

  lut_buffers[lut_id]      = lut;
  lut_buffer_sizes[lut_id] = width * height * bytes_per_cell;
}


vector<unsigned int>
FuseServerTCP::getLutIDs()
{
  vector< unsigned int > rv;
  rv.clear();

  std::map< unsigned int, unsigned int >::iterator it;

  for (it = lut_refcount.begin(); it != lut_refcount.end(); ++it) {
    rv.push_back( (*it).first );
  }

  return rv;
}


vector<unsigned int>
FuseServerTCP::getRequestedLutIDs()
{
  vector< unsigned int > rv;
  rv.clear();

  std::map< unsigned int, unsigned int >::iterator it;

  for (it = lut_refcount.begin(); it != lut_refcount.end(); ++it) {
    if ( lut_needs_refresh[(*it).first] ) {
      rv.push_back( (*it).first );
    }
  }

  return rv;
}


void
FuseServerTCP::setLutAvailable(unsigned int lut_id, bool available)
{
  if ( lut_available.find(lut_id) == lut_available.end() ) return;

  lut_available[lut_id] = available;
}


void
FuseServerTCP::setLutAppAlive(unsigned int lut_id,
			      bool alive,
			      std::string appl)
{
  if (lut_app.find( lut_id ) == lut_app.end()) return;
  if (lut_app_alive.find( lut_id ) == lut_app_alive.end()) return;

  lut_app[lut_id] = appl;
  lut_app_alive[lut_id] = alive;
}


void
FuseServerTCP::close_image(unsigned int image_num)
{
  if (image_num == IMAGE_ALL) {
    map< unsigned int, unsigned int >::iterator it;
    while ( (it = image_refcount.begin()) != image_refcount.end() ) {
      close_image( (*it).first );
    }
  } else {
    if ( (image_refcount.find(image_num) != image_refcount.end()) &&
	 (--image_refcount[image_num] == 0) ) {
      // no one is requesting this image, remove from requested images
      img_buffer_sizes.erase( image_num );
      if (image_copy_mode == FuseServer::DEEP_COPY) {
	free( img_buffers[image_num] );
      }
      img_buffers.erase( image_num );
      image_available.erase( image_num );
      image_needs_refresh.erase( image_num );
      image_headers.erase( image_num );
      image_refcount.erase( image_num );
      image_app.erase( image_num );
      image_app_alive.erase( image_num );
    }
  }
}


void
FuseServerTCP::open_image(unsigned int image_num)
{
  if ( image_refcount.find( image_num ) == image_refcount.end() ) {
    // the newly requested image is not loaded, add
    image_refcount[ image_num ] = 1;
    FUSE_image_packet_header_t ci;
    memset( &ci, 0, sizeof(ci));
    ci.image_num = htonl(image_num);
    image_headers[image_num]    = ci;
    img_buffers[image_num]      = NULL;
    image_available[image_num]  = false;
    image_needs_refresh[image_num] = true;
    image_app_alive[image_num]  = false;
    image_app[image_num]        = "";
    img_buffer_sizes[image_num] = 0;
  } else {
    ++image_refcount[image_num];
  }
}


void
FuseServerTCP::close_lut(unsigned int lut_id)
{
  if (lut_id == LUT_ALL) {
    map< unsigned int, unsigned int >::iterator it;
    while ( (it = lut_refcount.begin()) != lut_refcount.end() ) {
      close_lut( (*it).first );
    }
  } else {
    if ((lut_refcount.find(lut_id) != lut_refcount.end()) &&
	(--lut_refcount[lut_id] == 0)) {
      // no one is requesting this image, remove from requested images
      lut_buffer_sizes.erase( lut_id );
      lut_buffers.erase( lut_id );
      lut_headers.erase( lut_id );
      lut_refcount.erase( lut_id );
      lut_available.erase( lut_id );
      lut_needs_refresh.erase( lut_id );
      lut_app.erase( lut_id );
      lut_app_alive.erase( lut_id );
    }
  }

}


void
FuseServerTCP::open_lut(unsigned int lut_id)
{
  if ( lut_refcount.find( lut_id ) == lut_refcount.end() ) {
    // the newly requested image is not loaded, add
    lut_refcount[ lut_id ] = 1;
    FUSE_lookuptable_packet_header_t ch;
    memset( &ch, 0, sizeof(ch));
    ch.lut_id = lut_id;
    lut_headers[lut_id] = ch;
    lut_buffers[lut_id] = NULL;
    lut_available[lut_id] = false;
    lut_needs_refresh[lut_id] = true;
    lut_buffer_sizes[lut_id] = 0;
    lut_app[lut_id] = "";
    lut_app_alive[lut_id] = false;
  } else {
    ++lut_refcount[lut_id];
  }
}


std::vector<unsigned int>
FuseServerTCP::getUploadedLutIDs()
{
  vector< unsigned int > rv;
  rv.clear();

  std::map< unsigned int, unsigned int >::iterator it;

  for (it = lut_upload.begin(); it != lut_upload.end(); ++it) {
    rv.push_back( (*it).second );
  }

  return rv;
}


unsigned char *
FuseServerTCP::getUploadedLutBuffer(unsigned int lut_id)
{
  if (lut_upload_buffers.find( lut_id ) == lut_upload_buffers.end())  return NULL;

  return lut_upload_buffers[lut_id];
}


unsigned int
FuseServerTCP::getUploadedLutBufferSize(unsigned int lut_id)
{
  if (lut_upload_headers.find( lut_id ) == lut_upload_headers.end())  return 0;

  return ntohl(lut_upload_headers[lut_id].width) *
    ntohl(lut_upload_headers[lut_id].height) *
    ntohl(lut_upload_headers[lut_id].bytes_per_cell);
}


unsigned int
FuseServerTCP::getUploadedLutWidth(unsigned int lut_id)
{
  if (lut_upload_headers.find( lut_id ) == lut_upload_headers.end())  return 0;

  return ntohl(lut_upload_headers[lut_id].width);
}


unsigned int
FuseServerTCP::getUploadedLutHeight(unsigned int lut_id)
{
  if (lut_upload_headers.find( lut_id ) == lut_upload_headers.end())  return 0;

  return ntohl(lut_upload_headers[lut_id].height);
}


unsigned int
FuseServerTCP::getUploadedLutBytesPerCell(unsigned int lut_id)
{
  if (lut_upload_headers.find( lut_id ) == lut_upload_headers.end())  return 0;

  return ntohl(lut_upload_headers[lut_id].bytes_per_cell);
}


void
FuseServerTCP::setUploadLutSuccess(unsigned int lut_id, bool success)
{
  if (lut_upload_send_success.find( lut_id ) == lut_upload_send_success.end())  return;

  lut_upload_send_success[lut_id] = success;
}


void
FuseServerTCP::setUploadLutAppAlive(unsigned int lut_id,
				    bool alive,
				    std::string appl)
{
  if (lut_upload_app.find( lut_id ) == lut_upload_app.end()) return;
  if (lut_upload_app_alive.find( lut_id ) == lut_upload_app_alive.end()) return;

  lut_upload_app[lut_id] = appl;
  lut_upload_app_alive[lut_id] = alive;
}


bool
FuseServerTCP::isMessageAvailable()
{
  return (messages.size() > 0);
}


void
FuseServerTCP::getMessage(unsigned int *msgqid,
			  char **msg, unsigned int *msg_size)
{
  if (messages.size() == 0) throw Exception("No message available");

  FUSE_message_t *t = messages.front();
  *msgqid           = t->msgqid;
  *msg              = t->msg;
  *msg_size         = t->msg_size;
}


void
FuseServerTCP::freeMessage()
{
  free(messages.front()->msg);
  free(messages.front());
  messages.pop();
}


map< unsigned int, vector< pair< long, unsigned int > > >
FuseServerTCP::getMessageSubscriptions()
{
  map< unsigned int, vector< pair< long, unsigned int > > > rv;
  rv.clear();

  map< unsigned int, list< msg_subscr_t > >::iterator it;
  list< msg_subscr_t >::iterator  sit;

  for (it = msg_subscr.begin(); it != msg_subscr.end(); ++it) {
    vector< pair< long, unsigned int > > t;
    rv[(*it).first] = t;
    for (sit = (*it).second.begin(); sit != (*it).second.end(); ++sit) {
      pair< long, unsigned int > p((*sit).mtype, (*sit).data_size);
      rv[(*it).first].push_back( p );
    }
  }

  return rv;
}


void
FuseServerTCP::sendMessage(unsigned int msgqid, long mtype,
			   char *msg, unsigned int msg_size)
{

  if (msg_subscr.find( msgqid ) == msg_subscr.end()) {
    cout << "no subscription to that queue" << endl;
    return;
  }

  list< msg_subscr_t >::iterator it;

  for (it = msg_subscr[msgqid].begin(); it != msg_subscr[msgqid].end(); ++it) {

    if (mtype == (*it).mtype) {
      FUSE_message_packet_header_t msg_header;
      msg_header.msgqid = msgqid;
      msg_header.data_size = msg_size;

      // cout << "subcription found, sending to client " << (*it).client_id << endl;

      send_packet((*it).client_id, FUSE_PT_MESSAGE,
		  (unsigned char*)&msg_header,
		  sizeof(msg_header));

      send_data((*it).client_id, (unsigned char *)msg, msg_size);
    }
  }
}


void
FuseServerTCP::removeMessageQueueSubscriptions( unsigned int msgqid )
{
  if (msg_subscr.find( msgqid ) == msg_subscr.end()) {
    cout << "no subscription to that queue" << endl;
    return;
  }

  msg_subscr[msgqid].clear();
  msg_subscr.erase( msgqid );

}
