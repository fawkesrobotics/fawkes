
/***************************************************************************
 *  foutain.h - This header defines an image server
 *
 *  Generated: Thu Jan 26 13:48:27 2006
 *  Copyright  2005-2006  Tim Niemueller [www.niemueller.de]
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

#ifndef __FIREVISION_APPS_FOUNTAIN_FOUNTAIN_H_
#define __FIREVISION_APPS_FOUNTAIN_FOUNTAIN_H_

#include <utils/system/signal.h>

#include <string>
#include <map>

class Logger;
class ArgumentParser;
class SharedMemoryImageBuffer;
class SharedMemoryLookupTable;
class MiniImageProducer;
class IPCMessageQueue;
class FirevisionFountainBBClient;
class RCSoftXFountainConfig;
class FuseServerTCP;
class Scaler;

class Fountain : SignalHandler {
 public:
  Fountain(ArgumentParser *argp, Logger *logger,
	   unsigned int image_delay_sec, unsigned short int port);
  ~Fountain();

  void init();
  void run();
  void finalize();

  void handle_signal(int signum);

 private:
  std::string   applForImage(unsigned int image_num);
  std::string   applForLut(unsigned int lut_id);
  std::string   applForMsgQueue(unsigned int msgqid);
  bool          isFountainManagedMessageQueue(unsigned int msgqid);
  unsigned int  getImageIDforMiniImage(unsigned int image_id);
  unsigned int  image_id2num(const char *id);
  const char *  image_num2id(unsigned int num);

 private:

  ArgumentParser *argp;
  Logger *logger;
  bool quit;
  unsigned short int port;

  std::string  msg_prefix;
  int delay;

  FuseServerTCP  *fuses;
  std::map<unsigned int, SharedMemoryImageBuffer *> shm_images;
  std::map<unsigned int, SharedMemoryLookupTable *> shm_luts;
  std::map<unsigned int, SharedMemoryImageBuffer *>::iterator img_it;
  std::map<unsigned int, SharedMemoryLookupTable *>::iterator lut_it;

  std::map<unsigned int, MiniImageProducer *>       mini_image_producers;
  std::map<unsigned int, MiniImageProducer *>::iterator       mip_it;

  std::map<int, IPCMessageQueue *>            msg_queues;
  std::map<int, IPCMessageQueue *>::iterator  mit;
  int message_size;
  char *message;

  FirevisionFountainBBClient *bbclient;

  Scaler *scaler;
};



#endif
