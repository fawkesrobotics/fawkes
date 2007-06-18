/***************************************************************************
 *  fountain.cpp - Send images via the network
 *
 *  Generated: Wed Jan 11 13:53:22 2006
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

#include <apps/fountain/fountain.h>
#include <apps/fountain/bbclient.h>
#include <apps/fountain/mini_image_producer.h>

#include <core/exception.h>

#include <utils/system/argparser.h>
#include <utils/system/console_colors.h>
#include <utils/ipc/msg.h>
#include <utils/logging/logger.h>

#include <fvutils/color/colorspaces.h>
#include <fvutils/ipc/shm_registry.h>
#include <fvutils/ipc/shm_image.h>
#include <fvutils/ipc/shm_lut.h>
#include <fvutils/ipc/msg_registry.h>
#include <fvutils/net/fuse_server_tcp.h>
#include <fvutils/scalers/lossy.h>

// from linux/msg.h
#define MSGMAX 8192

using namespace std;

/** @class Fountain <apps/fountain/fountain.h>
 * Fountain Network Communicator.
 * Fountain provides access to FireVision internals via the network.
 * @author Tim Niemueller
 */

/** Constructor.
 * @param argp argument parser
 * @param logger Logger
 * @param image_delay_sec image delay in seconds
 * @param port network port to listen on
 */
Fountain::Fountain(ArgumentParser *argp, Logger *logger,
		   unsigned int image_delay_sec, unsigned short int port)
{
  this->argp   = argp;
  this->logger = logger;
  this->port   = port;

  quit = false;
  shm_images.clear();
  shm_luts.clear();
  msg_queues.clear();

  delay = image_delay_sec * 1000;

}


/** Destructor. */
Fountain::~Fountain()
{
}


/** Init. */
void
Fountain::init()
{

  cout << "Initializing Fuse Server.." << flush;

  fuses = new FuseServerTCP( port );
  fuses->setImageCopyMode( FuseServer::DEEP_COPY );

  cout << "done" << endl;

  if (! argp->hasArgument("b") ) {
    bbclient = new FirevisionFountainBBClient(argp->getArgC(), argp->getArgV());
    bbclient->Init();
    bbclient->addAliveClient( "firevision_front" );
    bbclient->addAliveClient( "firevision_omni" );
    bbclient->addAliveClient( "firevision_fusion" );
    bbclient->addAliveClient( "firevision_cruiser" );
  } else {
    bbclient = NULL;
  }

  message = (char *)malloc(MSGMAX);

  scaler = new LossyScaler();

  SignalManager::instance()->register_handler(SIGINT, this);
  SignalManager::instance()->ignore(SIGPIPE);

}


/** Finalize. */
void
Fountain::finalize()
{
  SignalManager::instance()->register_handler(SIGPIPE, NULL);
  SignalManager::instance()->finalize();

  for (img_it = shm_images.begin(); img_it != shm_images.end(); ++img_it) {
    delete (*img_it).second;
  }
  shm_images.clear();

  for (lut_it = shm_luts.begin(); lut_it != shm_luts.end(); ++lut_it) {
    delete (*lut_it).second;
  }
  shm_luts.clear();

  for (mit = msg_queues.begin(); mit != msg_queues.end(); ++mit) {
    delete (*mit).second;
  }
  msg_queues.clear();

  for (mip_it = mini_image_producers.begin(); mip_it != mini_image_producers.end(); ++mip_it) {
    delete (*mip_it).second;
  }
  mini_image_producers.clear();

  if (bbclient != NULL) {
    // delete bbclient;
    bbclient = NULL;
  }

  free(message);

  fuses->close();
  delete fuses;
}


void
Fountain::handle_signal(int signum)
{
  quit = true;
}


/** Run. */
void
Fountain::run()
{

  while (! quit) {

    // try to accept
    fuses->accept();

    // do a BB loop if we have a bbclient (aliveness)
    if (bbclient != NULL) bbclient->Loop(0);


    // check all opened images and LUTs if they still exist
    for (img_it = shm_images.begin(); img_it != shm_images.end(); ++img_it) {
      if ((*img_it).second->is_destroyed()) {
	// the controlling application quit and marked this segment as
	// destroyed, so we should close it and state it as "not existing"
	
	cout << msg_prefix << "Image shared memory segment "
	     << (*img_it).first << " got destroyed, closing" << endl;
	delete (*img_it).second;
	shm_images.erase( img_it );
      }
    }
    for (lut_it = shm_luts.begin(); lut_it != shm_luts.end(); ++lut_it) {
      if ((*lut_it).second->is_destroyed()) {
	// the controlling application quit and marked this segment as
	// destroyed, so we should close it and state it as "not existing"
	
	cout << msg_prefix << "Image shared memory segment "
	     << (*lut_it).first << " got destroyed, closing" << endl;
	delete (*lut_it).second;
	shm_luts.erase( lut_it );
      }
    }
    for (mit = msg_queues.begin(); mit != msg_queues.end(); ++mit) {
      if (! (*mit).second->isValid() ) {
	cout << msg_prefix << "Message queue " << (*mit).first << " got destroyed,"
	     << " closing" << endl;
	delete (*mit).second;
	msg_queues.erase( mit );
      }
    }

    if (quit) break;
    if (! fuses->connected()) {
      // if not connected sleep and try again
      usleep(delay);
      continue;
    }

    // process incoming messages
    fuses->process();

    if (quit) break;

    vector<unsigned int>            image_numbers;
    vector<unsigned int>            lut_ids;
    vector<unsigned int>            upload_lut_ids;
    vector<unsigned int>::iterator  it;
    map< unsigned int, vector< pair< long, unsigned int > > >  msg_subscripts;
    image_numbers   = fuses->getRequestedImageNumbers();
    lut_ids         = fuses->getRequestedLutIDs();
    upload_lut_ids  = fuses->getUploadedLutIDs();
    msg_subscripts  = fuses->getMessageSubscriptions();
    bool msgs_proc  = false;

    // process incoming messages
    while ( fuses->isMessageAvailable() ) {

      char         *msg;
      unsigned int  msg_size;
      unsigned int  msgqid;
 
      fuses->getMessage(&msgqid, &msg, &msg_size);

      /* important note:
       * calling isAlive here can be risky, it works only because of some special precautions
       * taken in the bbclient!
       * if we would call GetIsAlive from the alive interface every time, this WILL
       * fail (because the error count is growing, since the other application only updates
       * the alive integer every n ms, but we may be calling it very often in short period).
       * But bbclient queries _all_ alive clients in each Loop(). This is done to minimize
       * the problems where the alive status is checked for the first time. This only works
       * properly if you add the process with addAliveClient at the beginning.
       * the Alive_Client is flawed. A better way would be to have a timeout for the
       * application to be checked after it updated the BB the last time.
       */
      if ( (bbclient != NULL) && ! bbclient->isAlive( applForMsgQueue( msgqid ) ) ) {
	cout << msg_prefix << cyellow
	     << "Application " << applForMsgQueue( msgqid ) << " for message queue " << msgqid
	     << " is not alive." << cnormal << endl;
      } else {
	if (msg_queues.find( msgqid ) == msg_queues.end()) {
	  // msg queue not yet opened, do it
	  cout << "Trying to open message queue " << msgqid << ".." << flush;
	  bool destroy_on_delete = false;
	  bool create = false;
	  if (isFountainManagedMessageQueue(msgqid)) {
	    cout << "fountain controlled.." << flush;
	    destroy_on_delete = true;
	    create = true;
	  }
	  try {
	    IPCMessageQueue *mq = new IPCMessageQueue( msgqid, destroy_on_delete, create );
	    if (mq->isValid()) {
	      cout << "done" << endl;
	      msg_queues[msgqid] = mq;
	    } else {
	      cout << "failed" << endl;
	      delete mq;
	    }
	  } catch (Exception &e) {
	    cout << "failed" << endl;
	  }
	}

	if (msg_queues.find( msgqid ) != msg_queues.end()) {
	  IPCMessageQueue::MessageStruct *buf = (IPCMessageQueue::MessageStruct *)msg;
	  cout << msg_prefix << "enqueing message of type " << buf->mtype
	       << " into queue " << msgqid << endl;
	  msg_queues[msgqid]->send((IPCMessageQueue::MessageStruct *)msg, msg_size);
	} else {
	  cout << msg_prefix << cyellow << "queue " << msgqid << " not open, cannot send" << cnormal << endl;
	}
	msgs_proc = true;
      }

      fuses->freeMessage();
    }

    map< unsigned int, vector< pair< long, unsigned int > > >::iterator msit;
    vector< pair< long, unsigned int > >::iterator mtit;

    // open/create message queues for subscriptions
    for (msit = msg_subscripts.begin(); msit != msg_subscripts.end(); ++msit) {
      for (mtit = (*msit).second.begin(); mtit != (*msit).second.end(); ++mtit) {
	// cout << "checking for messages on queue " << (*msit).first << endl;
	if (msg_queues.find( (*msit).first ) == msg_queues.end()) {
	  // msg queue not yet opened, do it
	  cout << "Trying to open message queue " << (*msit).first << ".." << flush;
	  bool destroy_on_delete = false;
	  bool create = false;
	  if (isFountainManagedMessageQueue((*msit).first)) {
	    cout << "fountain controlled.." << flush;
	    destroy_on_delete = true;
	    create = true;
	  }
	  try {
	    IPCMessageQueue *mq = new IPCMessageQueue( (*msit).first, destroy_on_delete, create );
	    if (mq->isValid()) {
	      cout << "done" << endl;
	      msg_queues[(*msit).first] = mq;
	    } else {
	      cout << "failed" << endl;
	      delete mq;
	    }
	  } catch (Exception &e) {
	    cout << "failed" << endl;
	  }
	}

	if (msg_queues.find( (*msit).first ) != msg_queues.end()) {
	  if ( msg_queues[(*msit).first]->isValid() ) {

	    while (msg_queues[(*msit).first]->recv((*mtit).first,
						   (IPCMessageQueue::MessageStruct *)message,
						   (*mtit).second )) {
	      cout << "message has to be sent from local to remote"
		   << "  msgqid=" << (*msit).first
		   << "  mtype=" << (*mtit).first
		   << "  data_size=" << (*mtit).second
		   << endl;
	      fuses->sendMessage((*msit).first, (*mtit).first, message, (*mtit).second);
	    }
	    
	  } else {
	    delete msg_queues[(*msit).first];
	    msg_queues.erase((*msit).first);
	  }
	}
      }
    }

    // delete queues that do not have any subscriptions no more
    for (mit = msg_queues.begin(); mit != msg_queues.end(); ++mit) {
      if ( msg_subscripts.find( (*mit).first ) == msg_subscripts.end() ) {
	// no subscription for this message queue no more, delete
	delete (*mit).second;
	msg_queues.erase( mit );
      }
    }

    // if messages have been processed start loop from the beginning
    // message processing is handled prioritized!
    if (msgs_proc) {
      continue;
    }

    if (quit) break;

    string appl_for_image = "";

    // send out images
    for (it = image_numbers.begin(); it != image_numbers.end(); ++it) {

      appl_for_image = applForImage( *it );

      // if fountain is image deliverer, start mini image producer
      if ( appl_for_image == argp->getProgramName() ) {
	cout << msg_prefix << cyellow << "Fountain image requested" << cnormal << endl;
	unsigned int orig_image_id = getImageIDforMiniImage( *it );
	if ( orig_image_id == FIREVISION_SHM_IMAGE_INVALID ) {
	  fuses->setImageAppAlive( *it, true );
	  fuses->setImageAvailable( *it, false );
	} else {
	  if ( mini_image_producers.find( *it ) == mini_image_producers.end()) {
	    // no producer yet, start one!
	    MiniImageProducer *mip = new MiniImageProducer( image_num2id(orig_image_id),
							    image_num2id(*it),
							    scaler, logger );
	    if ( ! mip->isValid() ) {
	      cout << msg_prefix << cyellow << "Could not open mini image producer"
		   << cnormal << endl;
	      delete mip;
	    } else {
	      mini_image_producers[*it] = mip;
	      mip->produce();
	    }
	  } else {
	    mini_image_producers[*it]->produce();
	  }
	}
      }

      if ( (appl_for_image != argp->getProgramName()) &&
	   (bbclient != NULL) && ! bbclient->isAlive( appl_for_image ) ) {
	cout << msg_prefix << cyellow
	     << "Application " << applForImage( *it ) << " is not alive." << cnormal << endl;
	fuses->setImageAppAlive(  *it, false, appl_for_image );
	fuses->setImageAvailable( *it, false );
      } else {
	fuses->setImageAppAlive(  *it, true );

	if ( shm_images.find( *it ) == shm_images.end() ) {
	  // We do not yet have a shmem image buffer for that very image, try to open

	  cout << msg_prefix
	       << "Trying to open shmem segment for image " << *it << ".." << flush;
	  try {
	    SharedMemoryImageBuffer *img = new SharedMemoryImageBuffer( image_num2id(*it) );
	    
	    if (img->is_valid()) {
	      // colormap found
	      shm_images[*it] = img;
	      cout << "succeeded" << endl;
	    } else {
	      delete img;
	      cout << "failed" << endl;
	    }
	  } catch (Exception &e) {
	    cout << "failed" << endl;
	  }
	}

	if ( shm_images.find( *it ) != shm_images.end() ) {

	  if (shm_images[*it]->is_destroyed()) {
	    // the controlling application quit and marked this segment as
	    // destroyed, so we should close it and state it as "not existing"

	    cout << msg_prefix << "Image shared memory segment "
		 << *it << " got destroyed, closing and announcing death" << endl;
	    delete shm_images[*it];
	    shm_images.erase( *it );

	    fuses->setImageAvailable(   *it, false );

	  } else {
	    fuses->setImageColorspace(  *it, shm_images[*it]->colorspace() );
	    fuses->setImageWidth(       *it, shm_images[*it]->width() );
	    fuses->setImageHeight(      *it, shm_images[*it]->height() );
	    fuses->setImageCircleFound( *it, shm_images[*it]->circle_found() );
	    fuses->setImageCircle(      *it,
					shm_images[*it]->circle_x(),
					shm_images[*it]->circle_y(),
					shm_images[*it]->circle_radius() );
	    fuses->setImageROI(         *it,
					shm_images[*it]->roi_x(),
					shm_images[*it]->roi_y(),
					shm_images[*it]->roi_width(),
					shm_images[*it]->roi_height()
					);
	    fuses->setImageBuffer(      *it, shm_images[*it]->buffer());
	    fuses->setImageAvailable(   *it, true );
	  }
	} else {
	  fuses->setImageAvailable(   *it, false );
	}
      }
    }

    // send lookup tables
    for (it = upload_lut_ids.begin(); it != upload_lut_ids.end(); ++it) {

      if ( (bbclient != NULL) && ! bbclient->isAlive( applForLut( *it ) ) ) {
	cout << msg_prefix << cyellow
	     << "Application " << applForLut( *it ) << " is not alive." << cnormal << endl;
	fuses->setUploadLutAppAlive( *it, false, applForLut( *it ) );
	fuses->setUploadLutSuccess(  *it, false );
      } else {
	fuses->setUploadLutAppAlive( *it, true );

	if (shm_luts.find( *it ) == shm_luts.end() ) {
	  // we did not yet open the requested colormap, try
	  cout << msg_prefix
	       << "Trying to open shmem segment for lut " << *it << ".." << flush;
	  try {
	    SharedMemoryLookupTable *lut = new SharedMemoryLookupTable( *it,
									false /* read/write */);
	    if (lut->is_valid()) {
	      // colormap found
	      shm_luts[*it] = lut;
	      cout << "succeeded" << endl;
	    } else {
	      delete lut;
	      cout << "failed" << endl;
	    }
	  } catch (Exception &e) {
	    cout << "failed" << endl;
	  }
	}

	if (shm_luts.find( *it ) != shm_luts.end() ) {
	  
	  if (shm_luts[*it]->is_destroyed()) {
	    // the controlling application stopped and marked this segment as
	    // destroyed, so we should close it and state it as "not existing"
	    
	    cout << msg_prefix
		 << "Lookup table shared memory segment got destroyed, "
		 << "closing and announcing death" << endl;
	    delete shm_luts[*it];
	    shm_luts.erase( *it );

	    fuses->setUploadLutSuccess(*it, false);

	  } else {
	    if ( (shm_luts[*it]->getWidth() == fuses->getUploadedLutWidth(*it)) &&
		 (shm_luts[*it]->getHeight() == fuses->getUploadedLutHeight(*it)) &&
		 (shm_luts[*it]->getBytesPerCell() == fuses->getUploadedLutBytesPerCell(*it)) &&
		 (shm_luts[*it]->data_size() == fuses->getUploadedLutBufferSize(*it)) ) {
	      
	      shm_luts[*it]->set( fuses->getUploadedLutBuffer(*it) );
	      fuses->setUploadLutSuccess(*it, true);
	      
	    } else {
	      cout << msg_prefix << "Could not update local LUT " << *it
		   << ", inconsistent information:"
		   << endl
		   << "   local:  w=" << shm_luts[*it]->getWidth()
		   << "  h=" << shm_luts[*it]->getHeight()
		   << "  bpc=" << shm_luts[*it]->getBytesPerCell()
		   << endl
		   << "   remote: w=" << fuses->getUploadedLutWidth(*it)
		   << "  h=" << fuses->getUploadedLutHeight(*it)
		   << "  bpc=" << fuses->getUploadedLutBytesPerCell(*it)
		   << endl;
	      fuses->setUploadLutSuccess(*it, false);
	    }
	  }

	} else {
	  cout << msg_prefix << "Could not update local LUT " << *it
	       << ", lut not found" << endl;
	  fuses->setUploadLutSuccess(*it, false);
	}
      }
    }

    // download LUTs
    for (it = lut_ids.begin(); it != lut_ids.end(); ++it) {

      if ( (bbclient != NULL) && ! bbclient->isAlive( applForLut( *it ) ) ) {
	cout << msg_prefix << cyellow
	     << "Application " << applForLut( *it ) << " is not alive." << cnormal << endl;
	fuses->setLutAppAlive(  *it, false, applForLut( *it ) );
	fuses->setLutAvailable( *it, false );
      } else {
	fuses->setLutAppAlive( *it, true );

	if (shm_luts.find( *it ) == shm_luts.end() ) {
	  // we did not yet open the requested colormap, try
	  cout << msg_prefix
	       << "Trying to open shmem segment for lut " << *it << ".." << flush;
	  try {
	    SharedMemoryLookupTable *lut = new SharedMemoryLookupTable( *it,
									false /* read/write */);
	    if (lut->is_valid()) {
	      // colormap found
	      shm_luts[*it] = lut;
	      cout << "succeeded" << endl;
	    } else {
	      delete lut;
	      cout << "failed" << endl;
	    }
	  } catch (Exception &e) {
	    cout << "failed" << endl;
	  }
	}
	if (shm_luts.find( *it ) != shm_luts.end() ) {
	  
	  if (shm_luts[*it]->is_destroyed()) {
	    // the controlling application stopped and marked this segment as
	    // destroyed, so we should close it and state it as "not existing"
	  
	    cout << msg_prefix
		 << "Shared Memory Segmented got destroyed, closing and announcing death"
		 << endl;
	    delete shm_luts[*it];
	    shm_luts.erase( *it );

	    fuses->setUploadLutSuccess(*it, false);
	  } else {

	    fuses->setLUT( *it,
			   shm_luts[*it]->getWidth(),
			   shm_luts[*it]->getHeight(),
			   shm_luts[*it]->getBytesPerCell(),
			   shm_luts[*it]->getBuffer() );
	    fuses->setLutAvailable( *it, true );
	  }
	} else {
	  fuses->setLutAvailable( *it, false );
	}
      }
    }

    fuses->send();

    usleep(delay);

  }

  cout << endl;
  fuses->disconnect();
}


/** Map image numbers to applications.
 * @param image_num image number
 * @return application name
 */
std::string
Fountain::applForImage(unsigned int image_num)
{
  if ( (image_num > 0) && (image_num < 10) ) {
    return "firevision_front";
  } else if ( (image_num > 9) && (image_num < 20) ) {
    return "firevision_omni";
  } else if ( (image_num > 19) && (image_num < 40) ) {
    return "firevision_fountain";
  } else {
    return "";
  }
}


/** Map LUT IDs to applications.
 * @param lut_id LUT ID
 * @return application name
 */
std::string
Fountain::applForLut(unsigned int lut_id)
{
  if ( (lut_id > 0) && (lut_id < 50) ) {
    return "firevision_front";
  } else if ( (lut_id > 49) && (lut_id < 100) ) {
    return "firevision_front";
  } else {
    return "";
  }
}


/** Map message queue ID to application.
 * @param msgqid message queue ID
 * @return application name.
 */
std::string
Fountain::applForMsgQueue(unsigned int msgqid)
{
  if (msgqid == FIREVISION_MSGQ_CRUISER) {
    return "firevision_cruiser";
  } else {
    return "";
  }
}


/** Check if message queue is managed by fountain.
 * @param msgqid message queue ID
 * @return true if managed by Fountain, false otherwise
 */
bool
Fountain::isFountainManagedMessageQueue(unsigned int msgqid)
{
  return (
	  (msgqid == FIREVISION_MSGQ_FIELDINFO_FRONT) ||
	  (msgqid == FIREVISION_MSGQ_FIELDINFO_OMNI) ||
	  (msgqid == FIREVISION_MSGQ_FIELDINFO_FUSION)
	  );
}


/** Map Image number to mini image number.
 * @param image_id image number
 * @return mini image number
 */
unsigned int
Fountain::getImageIDforMiniImage(unsigned int image_id)
{
  if ( image_id == FIREVISION_SHM_IMAGE_FOUNTAIN_MINI_FRONT_RAW ) {
    return FIREVISION_SHM_IMAGE_FRONT_RAW;
  } else if ( image_id == FIREVISION_SHM_IMAGE_FOUNTAIN_MINI_FRONT_PROC ) {
    return FIREVISION_SHM_IMAGE_FRONT_PROCESSED;
  } else if ( image_id == FIREVISION_SHM_IMAGE_FOUNTAIN_MINI_OMNI_RAW ) {
    return FIREVISION_SHM_IMAGE_OMNI_RAW;
  } else {
    return FIREVISION_SHM_IMAGE_INVALID;
  }
}


/** Map image ID to image number.
 * @param id image id
 * @return image number
 */
unsigned int
Fountain::image_id2num(const char *id)
{
  if ( strcmp(id, "front-raw") == 0 ) {
    return FIREVISION_SHM_IMAGE_FRONT_RAW;
  } else if ( strcmp(id, "front-processed") == 0 ) {
    return FIREVISION_SHM_IMAGE_FRONT_PROCESSED;
  } else if ( strcmp(id, "geegaw-raw") == 0 ) {
    return FIREVISION_SHM_IMAGE_GEEGAW_RAW;
  } else if ( strcmp(id, "geegaw-processed") == 0 ) {
    return FIREVISION_SHM_IMAGE_GEEGAW_PROCESSED;
  } else if ( strcmp(id, "cannikin-raw") == 0 ) {
    return FIREVISION_SHM_IMAGE_CANNIKIN_RAW;
  } else if ( strcmp(id, "cannikin-processed") == 0 ) {
    return FIREVISION_SHM_IMAGE_CANNIKIN_PROCESSED;
  } else if ( strcmp(id, "omni-raw") == 0 ) {
    return FIREVISION_SHM_IMAGE_OMNI_RAW;
  } else if ( strcmp(id, "omni-processed") == 0 ) {
    return FIREVISION_SHM_IMAGE_OMNI_PROCESSED;
  } else if ( strcmp(id, "mini-front-raw") == 0 ) {
    return FIREVISION_SHM_IMAGE_FOUNTAIN_MINI_FRONT_RAW;
  } else if ( strcmp(id, "mini-front-proc") == 0 ) {
    return FIREVISION_SHM_IMAGE_FOUNTAIN_MINI_FRONT_PROC;
  } else if ( strcmp(id, "mini-omni-raw") == 0 ) {
    return FIREVISION_SHM_IMAGE_FOUNTAIN_MINI_OMNI_RAW;
  } else {
    return FIREVISION_SHM_IMAGE_INVALID;
  }
}


/** Map image number to image ID.
 * @param num image number
 * @return image ID
 */
const char *
Fountain::image_num2id(unsigned int num)
{
  switch (num) {
  case FIREVISION_SHM_IMAGE_FRONT_RAW:  return "front-raw";
  case FIREVISION_SHM_IMAGE_FRONT_PROCESSED:  return "front-processed";
  case FIREVISION_SHM_IMAGE_OMNI_RAW:  return "omni-raw";
  case FIREVISION_SHM_IMAGE_OMNI_PROCESSED:  return "omni-processed";
  case FIREVISION_SHM_IMAGE_GEEGAW_RAW:  return "geegaw-raw";
  case FIREVISION_SHM_IMAGE_GEEGAW_PROCESSED:  return "geegaw-processed";
  case FIREVISION_SHM_IMAGE_CANNIKIN_RAW:  return "cannikin-raw";
  case FIREVISION_SHM_IMAGE_CANNIKIN_PROCESSED:  return "cannikin-processed";
  case FIREVISION_SHM_IMAGE_FOUNTAIN_MINI_FRONT_RAW:  return "mini-front-raw";
  case FIREVISION_SHM_IMAGE_FOUNTAIN_MINI_FRONT_PROC:  return "mini-front-proc";
  case FIREVISION_SHM_IMAGE_FOUNTAIN_MINI_OMNI_RAW:  return "mini-omni-raw";
  default:
    return "";
  }
}
