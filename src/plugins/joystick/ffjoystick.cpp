
/***************************************************************************
 *  ffjoystick.cpp - Joystick app to provide a local joystick via a
 *                   RemoteBlackBoard connection.
 *
 *  Created: Sun Nov 23 01:19:54 2008
 *  Copyright  2006-2011  Tim Niemueller [www.niemueller.de]
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

#include "acquisition_thread.h"
#include "act_thread.h"
#include "remote_bb_poster.h"

#include <core/threading/thread.h>
#include <core/threading/wait_condition.h>
#include <core/exceptions/system.h>
#include <netcomm/fawkes/client.h>
#include <blackboard/remote.h>
#include <blackboard/interface_listener.h>
#include <utils/system/argparser.h>
#include <utils/system/signal.h>
#include <logging/console.h>
#include <netcomm/fawkes/client_handler.h>
#include <netcomm/socket/socket.h>

#include <cstring>
#include <cstdlib>
#include <cstdio>
#include <unistd.h>
#include <string>

#include <interfaces/JoystickInterface.h>

using namespace fawkes;

bool quit = false;

void
print_usage(const char *program_name)
{
  printf("Usage: %s [-h] [-r host[:port]] [-d device] [-l]\n"
	 " -h              This help message\n"
	 " -r host[:port]  Remote host (and optionally port) to connect to\n"
	 " -d device       Joystick device to use\n"
	 " -l              Start in logging mode - print data read from bb\n",
	 program_name);
}

/** Simple signal handler for ffjoystick.
 * @author Tim Niemueller
 */
class JoystickQuitHandler : public SignalHandler
{
 public:
  /** Constructor.
   * @param aqt Joystick acquisition thread
   */
  JoystickQuitHandler(JoystickAcquisitionThread &aqt)
    : __aqt(aqt)
  {
  }

  virtual void handle_signal(int signal)
  {
    __aqt.cancel();
  }

 private:
  JoystickAcquisitionThread &__aqt;
};

/** Log joystick data gathered via RemoteBlackBoard to console.
 * @author Tim Niemueller
 */
class JoystickBlackBoardLogger
  : public BlackBoardInterfaceListener,
    public SignalHandler
{
 public:
  /** Constructor.
   * @param argp argument parser
   * @param logger logger
   */
  JoystickBlackBoardLogger(ArgumentParser &argp, Logger *logger)
    :  BlackBoardInterfaceListener("JoystickBlackBoardLogger"),
       __logger(logger)
  {
    char *host = (char *)"localhost";
    unsigned short int port = 1910;
    bool free_host = argp.parse_hostport("r", &host, &port);
    
    __bb = new RemoteBlackBoard(host, port);
    if ( free_host )  free(host);

    __joystick_if = __bb->open_for_reading<JoystickInterface>("Joystick");
    __warning_printed = false;

    __joystick_if->read();
    logger->log_debug("Joystick", "Number of Axes:    %i",
                      __joystick_if->num_axes());
    logger->log_debug("Joystick", "Number of Buttons: %i",
                      __joystick_if->num_buttons());

    /** Just a quick hack for rumble testing
      __joystick_if->msgq_enqueue(
        new JoystickInterface::StartRumbleMessage(1000, 0,
                                                  JoystickInterface::DIRECTION_UP,
                                                  0xFFFF, 0x8000));
    */

    bbil_add_data_interface(__joystick_if);
    __bb->register_listener(this);
  }

  /** Destructor. */
  ~JoystickBlackBoardLogger()
  {
    __bb->close(__joystick_if);
    delete __bb;
  }

  virtual void bb_interface_data_changed(Interface *interface) throw()
  {
    if ( ! __bb->is_alive() ) {
      if ( __bb->try_aliveness_restore() ) {
	__logger->log_info("Joystick", "Connection re-established, writing data");
	__warning_printed = false;
      }
    }

    try {
      __joystick_if->read();
      float *axis_value = __joystick_if->axis();
      __logger->log_info("Joystick", "Axes:    0: %f  1: %f  2: %f  3: %f  4: %f  "
			 "5: %f  6: %f  7: %f",
			 axis_value[0], axis_value[1],
			 axis_value[2], axis_value[3],
			 axis_value[4], axis_value[5],
			 axis_value[6], axis_value[7]);
      char button_string[33];
      button_string[32] = 0;
      unsigned int pressed_buttons = __joystick_if->pressed_buttons();
      for (unsigned int i = 0; i < 32; ++i) {
	button_string[i] = (pressed_buttons & (1 << i)) ? '1' : '0';
      }
      __logger->log_info("Joystick", "Buttons: %s", button_string);
    } catch (Exception &e) {
      if ( ! __warning_printed ) {
	e.print_trace();
	__logger->log_warn("Joystick", "Lost connection to BlackBoard, "
                           "will try to re-establish");
	__warning_printed = true;
      }
    }
  }

  void handle_signal(int signum)
  {
    __waitcond.wake_all();
  }

  /** Wait for quit signal from signal handler. */
  void run()
  {
    __waitcond.wait();
  }

 private:
  bool __warning_printed;
  BlackBoard *__bb;
  Logger *__logger;
  JoystickInterface *__joystick_if;
  WaitCondition __waitcond;
};


/** Wake actuator thread on incomin messages.
 * @author Tim Niemueller
 */
class JoystickBlackBoardActListener
  : public BlackBoardInterfaceListener
{
 public:
  /** Constructor.
   * @param aqt acquisition thread to pass to message processor
   * @param blackboard blackboard to register for message event handling
   * @param joystick_if joystick interface to listen on for messages
   * @param logger logger
   */
  JoystickBlackBoardActListener(JoystickAcquisitionThread *aqt,
                                BlackBoard *blackboard,
                                JoystickInterface *joystick_if,
                                Logger *logger)
    :  BlackBoardInterfaceListener("JoystickBlackBoardActMsgProcThread"),
       __bb(blackboard), __joystick_if(joystick_if)
  {
    __msgproc = new JoystickActThread::MessageProcessor(aqt, __joystick_if);
    __msgproc->process();
    bbil_add_message_interface(__joystick_if);
    __bb->register_listener(this);
  }

  /** Destructor. */
  ~JoystickBlackBoardActListener()
  {
    __bb->unregister_listener(this);
    bbil_remove_message_interface(__joystick_if);
    delete __msgproc;
  }

  virtual bool bb_interface_message_received(Interface *interface,
                                             Message *message) throw()
  {
    try {
      __msgproc->process();
      __msgproc->process_message(message);
    } catch (Exception &e) {
      e.print_trace();
    }
    return false;
  }

 private:
  JoystickActThread::MessageProcessor *__msgproc;
  BlackBoard *__bb;
  JoystickInterface *__joystick_if;
};

/** Config tool main.
 * @param argc argument count
 * @param argv arguments
 */
int
main(int argc, char **argv)
{
  try
  {
    ArgumentParser argp(argc, argv, "hr:d:l");
    
    if ( argp.has_arg("h") ) {
      print_usage(argv[0]);
      exit(0);
    }

    const char *joystick_device = "/dev/input/js0";
    if ( argp.has_arg("d") ) {
      joystick_device = argp.arg("d");
    }

    ConsoleLogger logger;

    if ( argp.has_arg("l") ) {
      JoystickBlackBoardLogger jbl(argp, &logger);
      SignalManager::register_handler(SIGINT, &jbl);
      jbl.run();
    } else {
      char *host = (char *)"localhost";
      unsigned short int port = 1910;
      bool free_host = argp.parse_hostport("r", &host, &port);

      JoystickRemoteBlackBoardPoster jbp(host, port, &logger);
      JoystickAcquisitionThread aqt(joystick_device, &jbp, &logger);
      JoystickBlackBoardActListener aml(&aqt, jbp.blackboard(),
                                        jbp.joystick_if(), &logger);

      JoystickQuitHandler jqh(aqt);
      SignalManager::register_handler(SIGINT, &jqh);

      if (free_host)  free(host);

      aqt.start();
      aqt.join();
    }
  }
  catch (UnknownArgumentException &e)
  {
    printf("Error: Unknown Argument\n\n");
    print_usage(argv[0]);
    exit(0);
  }
  catch (SocketException &e)
  {
    printf("\nError: could not connect:\n%s\n", e.what());
  }
  catch (CouldNotOpenFileException &e)
  {
    printf("\nError: could not open joystick device:\n%s\n", e.what());
  }

  return 0;
}
