
/***************************************************************************
 *  qa_bb_notify.cpp - BlackBoard notification QA
 *
 *  Created: Mon Nov 12 14:35:53 2007
 *  Copyright  2006-2007  Tim Niemueller [www.niemueller.de]
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


/// @cond QA

#include <blackboard/blackboard.h>
#include <blackboard/remote.h>
#include <blackboard/exceptions.h>
#include <blackboard/bbconfig.h>
#include <blackboard/interface_listener.h>
#include <blackboard/interface_observer.h>

#include <interfaces/test.h>

#include <core/exceptions/system.h>
#include <utils/logging/liblogger.h>

#include <signal.h>
#include <unistd.h>

#include <iostream>
#include <vector>

using namespace std;

class QaBBEventListener
  : public BlackBoardInterfaceListener,
    public BlackBoardInterfaceObserver
{
 public:
  QaBBEventListener()
  {
    bbio_add_interface_create_type("TestInterface");
    bbio_add_interface_destroy_type("TestInterface");
  }

  virtual void bb_interface_created(const char *type, const char *id) throw()
  {
    printf("BBIO: Interface %s of type %s has been created\n", id, type);
  }

  virtual void bb_interface_destroyed(const char *type, const char *id) throw()
  {
    printf("BBIO: Interface %s of type %s has been destroyed\n", id, type);
  }


  virtual void bb_interface_data_changed(Interface *interface) throw()
  {
    printf("BBIL: Data in interface %s has been modified\n", interface->uid());
  }

  virtual void bb_interface_writer_added(Interface *interface) throw()
  {
    printf("BBIL: Writer has been added to interface %s\n", interface->uid());
  }

  virtual void bb_interface_writer_removed(Interface *interface) throw()
  {
    printf("BBIL: Writer has been removed from interface %s\n", interface->uid());
  }

  virtual void bb_interface_reader_added(Interface *interface) throw()
  {
    printf("BBIL: Reader has been added to interface %s\n", interface->uid());
  }

  virtual void bb_interface_reader_removed(Interface *interface) throw()
  {
    printf("BBIL: Reader has been removed from interface %s\n", interface->uid());
  }

  virtual void add_interface(Interface *interface) throw()
  {
    printf("Listener: Adding interface %s\n", interface->uid());
    bbil_add_data_interface(interface);
    bbil_add_reader_interface(interface);
    bbil_add_writer_interface(interface);
  }
};


int
main(int argc, char **argv)
{
  LibLogger::init();

  RemoteBlackBoard *bb = new RemoteBlackBoard("localhost", 1910);
  //BlackBoard *bb = new BlackBoard();

  QaBBEventListener qabbel;

  TestInterface *ti_writer_1;
  TestInterface *ti_writer_2;
  TestInterface *ti_writer_3;
  TestInterface *ti_writer_4;
  TestInterface *ti_writer_5;
  TestInterface *ti_writer_6;

  try {
    cout << "Opening interfaces.. " << endl;
    ti_writer_1 = bb->open_for_writing<TestInterface>("SomeID 1");
    ti_writer_2 = bb->open_for_writing<TestInterface>("SomeID 2");

    qabbel.add_interface(ti_writer_1);
    qabbel.add_interface(ti_writer_2);
    bb->register_listener(&qabbel, BlackBoard::BBIL_FLAG_ALL);
    bb->register_observer(&qabbel, BlackBoard::BBIO_FLAG_ALL);

    ti_writer_3 = bb->open_for_writing<TestInterface>("SomeID 3");
    ti_writer_4 = bb->open_for_writing<TestInterface>("AnotherID 1");
    ti_writer_5 = bb->open_for_writing<TestInterface>("AnotherID 2");
    ti_writer_6 = bb->open_for_writing<TestInterface>("AnotherID 3");
    cout << "success" << endl;
  } catch (Exception &e) {
    cout << "failed! Aborting" << endl;
    e.print_trace();
    exit(1);
  }

  usleep(100000);

  std::list<TestInterface *> *readers = bb->open_all_of_type_for_reading<TestInterface>();
  usleep(100000);
  for (std::list<TestInterface *>::iterator i = readers->begin(); i != readers->end(); ++i) {
    printf("Opened reader for interface %s of type %s\n", (*i)->id(), (*i)->type());
    bb->close(*i);
  }
  delete readers;

  usleep(100000);

  const char* prefix = "Another";
  readers = bb->open_all_of_type_for_reading<TestInterface>(prefix);
  printf("Found %zu interfaces with prefix \"%s\"\n", readers->size(), prefix);
  for (std::list<TestInterface *>::iterator i = readers->begin(); i != readers->end(); ++i) {
    printf("Opened reader for interface %s of type %s\n", (*i)->id(), (*i)->type());
    bb->close(*i);
  }
  delete readers;

  usleep(100000);

  printf("Removing writer one. This should print a warning.\n");
  bb->close(ti_writer_1);
  bb->unregister_listener(&qabbel);

  usleep(100000);

  printf("Removing other writers. No warning should appear.\n");
  bb->close(ti_writer_2);
  bb->close(ti_writer_3);
  bb->close(ti_writer_4);
  bb->close(ti_writer_5);
  bb->close(ti_writer_6);

  usleep(100000);

  delete bb;
  LibLogger::finalize();
}


/// @endcond
