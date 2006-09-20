
/***************************************************************************
 *  exception.cpp - basic exception
 *
 *  Generated: Thu Feb 09 13:04:45 2006 (from FireVision)
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

#include <core/exception.h>
#include <core/threading/mutex.h>

#include <string.h>
#include <stdlib.h>
#include <stdio.h>

Exception::Exception(const char *msg)
{
  messages_mutex = new Mutex();
  messages_mutex->lock();

  messages = NULL;
  messages_end = NULL;
  messages_iterator = NULL;

  if ( msg != NULL ) {
    append_nolock(msg);
  } else {
    append_nolock("Basic Exception");
  }

  messages_mutex->unlock();
}

Exception::Exception(const Exception &exc)
{
  messages_mutex = new Mutex();

  copy_messages(exc);
}

Exception::Exception()
{
  messages = NULL;
  messages_end = NULL;
  messages_iterator = NULL;
}


Exception::~Exception()
{
  message_list_t *msg_this;
  messages_iterator = messages;
  while ( messages_iterator ) {
    free(messages_iterator->msg);
    msg_this = messages_iterator;
    messages_iterator = messages_iterator->next;
    free(msg_this);
  }
  messages = NULL;
  messages_end = NULL;
  delete messages_mutex;
}


void
Exception::append(const char *msg)
{
  // do not append empty messages
  if (msg == NULL)  return;

  messages_mutex->lock();
  append_nolock(msg);
  messages_mutex->unlock();
}


void
Exception::append_nolock(const char *msg)
{
  if ( messages == NULL ) {
    // This is our first message
    messages = (message_list_t *)malloc(sizeof(message_list_t));
    messages->next = NULL;
    messages->msg  = strdup(msg);
    messages_end = messages;
  } else {
    message_list_t *ml = (message_list_t *)malloc(sizeof(message_list_t));
    ml->next = NULL;
    ml->msg = strdup(msg);
    messages_end->next = ml;
    messages_end = ml;
  }
}


void
Exception::append_nocopy(char *msg)
{
  if ( messages == NULL ) {
    // This is our first message
    messages = (message_list_t *)malloc(sizeof(message_list_t));
    messages->next = NULL;
    messages->msg  = msg;
    messages_end = messages;
  } else {
    message_list_t *ml = (message_list_t *)malloc(sizeof(message_list_t));
    ml->next = NULL;
    ml->msg = msg;
    messages_end->next = ml;
    messages_end = ml;
  }  
}


Exception &
Exception::operator=(const Exception &exc)
{
  messages_mutex = new Mutex();
  copy_messages(exc);

  return *this;
}

void
Exception::copy_messages(const Exception &exc)
{
  messages_mutex->lock();
  exc.messages_mutex->lock();

  // copy messages
  messages_iterator = exc.messages;
  while ( messages_iterator ) {
    append_nolock(messages_iterator->msg);
    messages_iterator = messages_iterator->next;
  }

  exc.messages_mutex->unlock();
  messages_mutex->unlock();
}


const char *
Exception::c_str()
{
  if ( messages != NULL ) {
    return messages->msg;
  } else {
    return "No exception message available";
  }
}


void
Exception::raise()
{
  throw *this;
}


void
Exception::printTrace()
{
  messages_mutex->lock();
  fprintf(stderr, "Exception trace\n"
	  "============================================================================\n");
  if ( messages == NULL ) {
    fprintf(stderr, "No messages recorded.\n");
  } else {
    messages_iterator = messages;
    while ( messages_iterator ) {
      fprintf(stderr, "%s\n", messages_iterator->msg);
      messages_iterator = messages_iterator->next;
    }
  }
  fprintf(stderr,
	  "============================================================================\n");
  messages_mutex->unlock();
}
