
/***************************************************************************
 *  fuse_server_client_thread.h - client thread for FuseServer
 *
 *  Created: Tue Nov 13 19:59:11 2007
 *  Copyright  2005-2007  Tim Niemueller [www.niemueller.de]
 *
 *  $Id$
 *
 ****************************************************************************/

/*  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version. A runtime exception applies to
 *  this software (see LICENSE file mentioned below for details).
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU Library General Public License for more details.
 *
 *  You can read the full text in the LICENSE file in the doc directory. 
 */

#ifndef __FIREVISION_FVUTILS_NET_FUSE_SERVER_CLIENT_THREAD_H_
#define __FIREVISION_FVUTILS_NET_FUSE_SERVER_CLIENT_THREAD_H_

#include <core/threading/thread.h>

#include <map>
#include <string>

class FuseServer;
class StreamSocket;
class FuseNetworkMessageQueue;
class FuseNetworkMessage;
class SharedMemoryImageBuffer;
class SharedMemoryLookupTable;
class JpegImageCompressor;

class FuseServerClientThread : public Thread
{
 public:
  FuseServerClientThread(FuseServer *fuse_server, StreamSocket *s);
  virtual ~FuseServerClientThread();

  void recv();
  void send();
  virtual void loop();
  
  void process_greeting_message(FuseNetworkMessage *m);
  void process_getimage_message(FuseNetworkMessage *m);
  void process_getlut_message(FuseNetworkMessage *m);
  void process_getimagelist_message(FuseNetworkMessage *m);
  void process_getlutlist_message(FuseNetworkMessage *m);

 private:
  void process_inbound();


  FuseServer   *__fuse_server;
  StreamSocket *__socket;

  FuseNetworkMessageQueue *__outbound_queue;
  FuseNetworkMessageQueue *__inbound_queue;

  JpegImageCompressor *__jpeg_compressor;

  std::map< std::string, SharedMemoryImageBuffer * >  __buffers;
  std::map< std::string, SharedMemoryImageBuffer * >::iterator  __bit;

  std::map< std::string, SharedMemoryLookupTable * >  __luts;
  std::map< std::string, SharedMemoryLookupTable * >::iterator  __lit;

  bool __alive;
};


#endif
