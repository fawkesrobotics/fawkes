
/***************************************************************************
 *  qa_socket_stream.cpp - Fawkes QA StreamSocket
 *
 *  Created: Fri Nov 11 14:38:10 2006 (on train back from Google, Hamburg)
 *  Copyright  2006  Tim Niemueller [www.niemueller.de]
 *
 ****************************************************************************/

/*  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version. A runtime exception applies to
 *  this software (see LICENSE.GPL_WRE file mentioned below for details).
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU Library General Public License for more details.
 *
 *  Read the full text in the LICENSE.GPL_WRE file in the doc directory.
 */

/// @cond QA

#include <core/threading/thread.h>
#include <netcomm/socket/stream.h>
#include <utils/system/signal.h>
#include <utils/system/argparser.h>

#include <cstdio>

using namespace fawkes;

class StreamServerThread : public Thread
{
public:
  StreamServerThread()
    : Thread("StreamServerThread", Thread::OPMODE_CONTINUOUS)
  {
    i = 0;
    s = new StreamSocket();
    s->bind(1910);
    s->listen();
    accepted = false;
    cs = NULL;
  }

  ~StreamServerThread()
  {
    printf("Closing server socket\n");
    s->close();
    if (cs) cs->close();
    printf("Closed server socket\n");
    delete s;
    delete cs;
  }

  virtual void loop()
  {
    if (! accepted) {
      try {
	cs = s->accept();
	accepted = true;
	printf("Server MTU: %u\n", cs->mtu());
      } catch (SocketException &e) {
	e.print_trace();
      }
    }
    if ( accepted ) {
      try {
	cs->write(&i, sizeof(i));
	unsigned int ri = 0;
	cs->read(&ri, sizeof(ri));
	if ( ri != i ) {
	  printf("ERROR: sent %u but received %u\n", i, ri);
	} else {
	  printf("OK: sent %u and received %u\n", i, ri);
	}
	++i;
      } catch (SocketException &e) {
	e.print_trace();
	printf("Loop failed, disconnecting and waiting for new connection\n");
	delete cs;
	cs = NULL;
	accepted = false;
      }
    }
  }

 private:
  unsigned int i;
  StreamSocket *s;
  Socket *cs;
  bool accepted;
};


class StreamClientThread : public Thread
{
public:
  StreamClientThread(const char *host)
    : Thread("StreamClientThread", Thread::OPMODE_CONTINUOUS)
  {
    if ( host == NULL ) {
      this->host = "127.0.0.1";
    } else {
      this->host = host;
    }

    s = new StreamSocket();
    connected = false;
  }

  ~StreamClientThread()
  {
    printf("Closing client socket\n");
    s->close();
    printf("Closed client socket\n");
    delete s;
  }

  virtual void loop()
  {
    if (! connected) {
      try {
	s->connect(host, 1910);
	connected = true;
	printf("Client MTU: %u\n", s->mtu());
      } catch (SocketException &e) {
	e.print_trace();
      }
    }
    if ( connected ) {
      unsigned int i = 0;
      s->read(&i, sizeof(i));
      s->write(&i, sizeof(i));
    }
  }

 private:
  const char *host;
  StreamSocket *s;
  bool connected;
};


class StreamSocketQAMain : public SignalHandler
{
 public:
  static const unsigned int MODE_STANDALONE = 1;
  static const unsigned int MODE_SERVER = 2;
  static const unsigned int MODE_CLIENT = 3;

  StreamSocketQAMain(unsigned int mode, const char *host = NULL)
  {
    s = NULL;
    c = NULL;
    if ( (mode == MODE_STANDALONE) ||
	 (mode == MODE_SERVER) ) {
      s = new StreamServerThread();
    }
    if ( (mode == MODE_STANDALONE) ||
	 (mode == MODE_CLIENT) ) {
      c = new StreamClientThread(host);
    }
  }

  ~StreamSocketQAMain()
  {
    delete s;
    delete c;
  }


  virtual void handle_signal(int signum)
  {
    printf("Signal received, cancelling threads\n");
    if (s)  s->cancel();
    if (c)  c->cancel();
    printf("Threads cancelled\n");
  }

  void run()
  {
    if (s)  s->start();
    if (c)  c->start();
    if (s)  s->join();
    if (c)  c->join();
  }

 private:
  StreamServerThread *s;
  StreamClientThread *c;

};

int
main(int argc, char **argv)
{
  StreamSocketQAMain *m;
  SignalManager::ignore(SIGPIPE);

  ArgumentParser argp(argc, argv, "sc:");

  if ( argp.has_arg("s") || argp.has_arg("c") ) {
    // Special mode
    if ( argp.has_arg("s") ) {
      // Only run Server
      m = new StreamSocketQAMain(StreamSocketQAMain::MODE_SERVER);
    } else {
      m = new StreamSocketQAMain(StreamSocketQAMain::MODE_CLIENT,
				 argp.arg("c"));
    }
  } else {
    m = new StreamSocketQAMain(StreamSocketQAMain::MODE_STANDALONE);
  }

  SignalManager::register_handler(SIGINT, m);

  m->run();
  delete m;

  SignalManager::finalize();
}

/// @endcond
