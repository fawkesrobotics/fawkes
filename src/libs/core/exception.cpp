
/***************************************************************************
 *  exception.cpp - basic exception
 *
 *  Generated: Thu Feb 09 13:04:45 2006 (from FireVision)
 *  Copyright  2005-2006  Tim Niemueller [www.niemueller.de]
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

#include <core/exception.h>
#include <core/threading/mutex.h>

#ifndef _GNU_SOURCE
#define _GNU_SOURCE
#endif

#include <cstring>
#include <cstdlib>
#include <cstdio>
#ifdef HAVE_EXECINFO
#  include <execinfo.h>
#endif

namespace fawkes {

/** @class Exception core/exception.h
 * Base class for exceptions in Fawkes.
 * Exceptions are a good way to handle errors. If you choose to use
 * exceptions use derivates of this class so that there is a unified way of
 * handling errors in Fawkes. Do <i>not</i> throw an arbitrary class such as
 * a string or integer as this is hard to handle.
 *
 * For your exceptions in general you only need to override the constructor
 * and call the Exception constructor with the appropriate message. In
 * cases where more information is needed about the error add appropriate
 * handlers.
 *
 * In most cases it is bad to just throw an Exception like this:
 *
 * @code
 * if ( error_condition ) {
 *   throw Exception("Out of memory");
 * }
 * @endcode
 *
 * Rather you should explicitly subclass Exception appropriately. For the
 * above example you could have something like this as exception class:
 *
 * @code
 * class OutOfMemException : public Exception
 * {
 *  public:
 *   OutOfMemoryException() : Exception("Out of memory") {}
 * }
 * @endcode
 *
 * And in your handling code you throw a OutOfMemoryException. This is
 * especially useful if it is possible to throw several different exceptions.
 * If the message was different you would have to parse the string for
 * the exact error. This can be avoided if you just catch different
 * exceptions. This is also useful if the Exception is not catched explicitly
 * as this will printout the name of the exception class thrown just before
 * exiting the program. And reading something like
 * "terminate called after throwing an instance of 'OutOfMemoryException'"
 * makes it a lot easier to spot the problem.
 *
 * Exceptions should be catched by reference like this:
 * @code
 * try {
 *   some_operation();
 * } catch (OutOfMemoryException &e) {
 *   std::cout << e.c_str() << std::endl;
 *   error_handling();
 * }
 * @endcode
 *
 * Messages are stored as list. The first message however is called the
 * primary message and it should contain as much information as available.
 * This message is printed on the screen if the application crashes with an
 * unhandled exception. So having meaningful content here means that the
 * error can be traced more easily.
 *
 * You can utilize the list feature by adding appropriate information
 * through appropriate try/catch statements. This way you can
 * build information path ways that will help to debug your software. Use
 * block like this to append information:
 * @code
 * try {
 *   potentially_failing();
 * } catch {MyException &e) {
 *   e.append("info where exception happened");
 *   throw; // re-throw exception
 * }
 * @endcode
 * This is especially useful if the exception may occur at several different
 * places and it cannot be fixed where it happens.
 *
 *
 * @see example_exception.cpp
 * @ingroup FCL
 * @ingroup Exceptions
 *
 * @author Tim Niemueller
 */
/** @var Exception::messages
 * List of messages. Should not be NULL. Messages are append with append().
 * Using a custom list to avoid including STL stuff in this core file.
 * @see append()
 */
/** @var Exception::messages_iterator
 * Iterator to iterate over messages
 */
/** @var Exception::messages_end
 * Pointer that points to the very last message. Used for fast appending.
 */
/** @var Exception::messages_mutex
 * Mutex to protect operations on messages list.
 */
/** @var Exception::_errno
 * Error number, should be used if the error was caused by a method that supplies
 * errno.
 */


/** Constructor.
 * Constructs a new exception with the given message.
 * @param format The format of the primary message. Supports the same
 * arguments as append(). The message is copied and not just referenced.
 * Thus the memory has to be freed if it is a dynamic  string on the heap.
 */
Exception::Exception(const char *format, ...) throw()
{ 
  messages_mutex = new Mutex();

  _errno = 0;
  __type_id = "unknown";

  messages = NULL;
  messages_end = NULL;
  messages_iterator = NULL;

  if ( format != NULL ) {
    va_list arg;
    va_start(arg, format);
    append_nolock_va(format, arg);
    va_end(arg);
  } else {
    append_nolock("Unnkown Exception");
  }
}


/** Constructor.
 * Constructs a new exception with the given message and errno value. This
 * is particularly handy when throwing the exception after a function failed
 * that returns an error code in errno. 
 * @param errno error number
 * @param format The format of the primary message. Supports the same
 * arguments as append(). The message is copied and not just referenced.
 * Thus the memory has to be freed if it is a dynamic  string on the heap.
 */
Exception::Exception(int errno, const char *format, ...) throw()
{
  messages_mutex = new Mutex();

  _errno = errno;
  __type_id = "unknown";

  messages = NULL;
  messages_end = NULL;
  messages_iterator = NULL;

  if ( format != NULL ) {
    va_list arg;
    va_start(arg, format);
    char *ext_format;
    if ( asprintf(&ext_format, "%s (errno: %i, %s)", format, errno, strerror(errno)) == -1 ) {
      append_nolock_va(format, arg);
    } else {
      append_nolock_va(ext_format, arg);
      free(ext_format);
    }
    va_end(arg);
  } else {
    append_nolock("Exception with errno=%i (%s)", errno, strerror(errno));
  }
}


/** Copy constructor.
 * The copy constructor is worth some extra discussion. If you do an exception
 * by value (which you shouldn't in the first place since this will generate a
 * copy, only do this if you can't avoid it for some reason. Not if you only
 * THINK that you can't avoid it) the copy constructor is called. If your catch
 * statements reads like
 * @code
 *   try {
 *     ...
 *   } catch (Exception e) {
 *     ...
 *   }
 * @endcode
 * then a copy will be created for the catch block. You throw the exception with
 * something like
 * @code
 *   throw Exception("Boom");
 * @endcode
 * This will create an Exception which is valid in the block where you throw the
 * exception. Now for the catch block a copy is created. Since the exception
 * holds a pointer on the heap the implicit copy constructor would just copy
 * the pointer, not the data. So both exceptions point to the same data (to the
 * message for the base exception). If now both destructors for the exception
 * are called they both try to free the very same memory. Of course the second
 * destructor will cause a disaster. If you are lucky your glibc detectes the
 * problem an kills the application. If you are not that fortunate you will
 * cause very strange behaviour of your application.
 *
 * In general you should not have to worry about this. But if you choose to have
 * own storage on the heap using either new, malloc or a method that returns
 * memory on the heap (like strdup()) you have to write your own copy contructor
 * and copy the memory area or take care that only one exception frees the memory.
 * @param exc Exception to copy
 */
Exception::Exception(const Exception &exc) throw()
{
  messages_mutex = new Mutex();

  messages = NULL;
  messages_end = NULL;
  messages_iterator = NULL;

  _errno = exc._errno;
  __type_id = exc.__type_id;
  copy_messages(exc);
}


/** Constructor for subclasses.
 * This constructor can be used in subclasses is some processing code is
 * needed (like sprintf) to assign the message. At least assign the empty
 * string to the message.
 */
Exception::Exception() throw()
{
  messages_mutex = new Mutex();
  _errno = 0;
  __type_id = "unknown";
  messages = NULL;
  messages_end = NULL;
  messages_iterator = NULL;
}


/** Destructor. */
Exception::~Exception() throw()
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


/** Set exception type ID.
 * Set the type ID of this exception.
 * @param id new type ID, note that this must be a static string which is
 * guaranteed to exist for the whole lifetime of the exception.
 * @see Exception::type_id()
 */
void
Exception::set_type_id(const char *id)
{
  __type_id = id;
}


/** Get type ID.
 * Exceptions can have a type ID. This can be used to avoid having to declare
 * numerous specific exception sub-classes to different errors, if it is
 * essential to be able to differentiate them in the exception handling code.
 * The type ID is a free-form string. It should NOT contain any message, rather
 * it should be a one-word internal identifier that is never leaked to the user
 * of the software, i.e. it is not printed anywhere. Note that the ID must be
 * a static string, which exists for the whole life time of the exception, is
 * generally not in a dynamically allocated memory (this very exception could
 * indicate memory shortage). This also makes it thread-safe.
 * @return type ID
 */
const char *
Exception::type_id() const
{
  return __type_id;
}


/** Prepend messages to the message list.
 * @param format format of the message to prepend, see printf(3) for details about formatting
 * options.
 */
void
Exception::prepend(const char *format, ...) throw()
{
  // do not append empty messages
  if (format == NULL)  return;

  va_list arg;
  va_start(arg, format);
  messages_mutex->lock();
  prepend_nolock_va(format, arg);
  messages_mutex->unlock();
  va_end(arg);
}


/** Append messages to the message list.
 * @param format format of the message to append, see printf(3) for details about formatting
 * options.
 */
void
Exception::append(const char *format, ...) throw()
{
  // do not append empty messages
  if (format == NULL)  return;

  va_list arg;
  va_start(arg, format);
  messages_mutex->lock();
  append_nolock_va(format, arg);
  messages_mutex->unlock();
  va_end(arg);
}


/** Append messages to the message list.
 * @param format format of the message to append, see printf(3) for details about formatting
 * options.
 * @param va va_list with arguments matching the format
 */
void
Exception::append_va(const char *format, va_list va) throw()
{
  // do not append empty messages
  if (format == NULL)  return;

  messages_mutex->lock();
  append_nolock_va(format, va);
  messages_mutex->unlock();
}


/** Append message that are from another Exception.
 * @param e Exception to copy messages from
 */
void
Exception::append(const Exception &e) throw()
{
  copy_messages(e);  
}


/** Append messages without lock.
 * this can be used to append messages without locking the mutex if the mutex
 * has been locked already to append many messages and keep their order intact
 * and thus to prevent messages to be appended inbetween.
 * Used for example in copy constructor.
 * @param format The format of the primary message. Supports the same
 * arguments as append(). The message is copied and not just referenced.
 * Thus the memory has to be freed if it is a dynamic  string on the heap.
 */
void
Exception::append_nolock(const char *format, ...) throw()
{
  va_list arg;
  va_start(arg, format);

  char *msg;
  if ( vasprintf(&msg, format, arg) == -1 ) {
    msg = strdup(format);
  }

  va_end(arg);

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


/** Prepend messages without lock by formatted string.
 * This can be used to append messages without locking the mutex if the mutex
 * has been locked already to append many messages and keep their order intact
 * and thus to prevent messages to be appended inbetween.
 * Used for example in copy constructor.
 * @param format format of the message to be appended
 * @param ap argument va_list for format
 */
void
Exception::prepend_nolock_va(const char *format, va_list ap) throw()
{
  char *msg;
  if ( vasprintf(&msg, format, ap) == -1 ) {
    msg = strdup(format);
  }

  if ( messages == NULL ) {
    // This is our first message
    messages = (message_list_t *)malloc(sizeof(message_list_t));
    messages->next = NULL;
    messages->msg  = msg;
    messages_end = messages;
  } else {
    message_list_t *ml = (message_list_t *)malloc(sizeof(message_list_t));
    ml->next = messages;
    ml->msg = msg;
    messages = ml;
  }
}


/** Append messages without lock by formatted string.
 * this can be used to append messages without locking the mutex if the mutex
 * has been locked already to append many messages and keep their order intact
 * and thus to prevent messages to be appended inbetween.
 * Used for example in copy constructor.
 * @param format format of the message to be appended
 * @param ap argument va_list for format
 */
void
Exception::append_nolock_va(const char *format, va_list ap) throw()
{
  char *msg;
  if ( vasprintf(&msg, format, ap) == -1 ) {
    msg = strdup(format);
  }

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


/** Append message without copying.
 * Can be used in subclasses to append messages that have been allocated
 * on the heap. Use with extreme care. Do not add constant strings! This would
 * cause your application to crash since the destructor will try to free all
 * messages. The message list is not locked.
 * @param msg Message to append.
 */
void
Exception::append_nolock_nocopy(char *msg) throw()
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


/** Assign an Exception.
 * As this is one of the Big Three (see C++ FAQ at
 * http://www.parashift.com/c++-faq-lite/coding-standards.html#faq-27.10) this
 * is needed because we already need a copy constructor. Read about the
 * copy constructor why this is the case.
 * @see Exception(const Exception &exc)
 * @param exc The exception with the values to assign to this exception.
 * @return reference to this object. Allows assignment chaining.
 */
Exception &
Exception::operator=(const Exception &exc) throw()
{
  messages_mutex = new Mutex();
  copy_messages(exc);

  return *this;
}


/** Copy messages from given exception.
 * Copies the messages from exc to this exception.
 * @param exc Exception to copy messages from.
 */
void
Exception::copy_messages(const Exception &exc) throw()
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


/** This can be used to throw this exception.
 * This can be used to throw this exception instance. This is a precaution if
 * it is needed. See C++ FAQ 17.10.
 */
void
Exception::raise()
{
  throw *this;
}


/** Prints a backtrace. */
void
Exception::print_backtrace() const throw()
{
#ifdef HAVE_EXECINFO
  void * array[25];
  int size = backtrace(array, 25);
  char ** symbols = backtrace_symbols(array, size);

  printf("Backtrace:\n");
  for (int i = 0; i < size; ++i) {
    printf("  %s\n", symbols[i]);
  }
  
  free(symbols);
#else
  printf("Backtrace not available on current system\n");
#endif
}


/** Generate backtrace string.
 * @return freshly allocated string of backtrace. Free after you are done.
 */
char *
Exception::generate_backtrace() const throw()
{
#ifdef HAVE_BACKTRACE
  void * array[25];
  int size = backtrace(array, 25);
  char ** symbols = backtrace_symbols(array, size);
  
  size_t total_size = 1; //null termination
  for (int i = 0; i < size; ++i) {
    total_size += strlen(symbols[i]) + 1;
  }
  char *rv = (char *)calloc(1, total_size);
  char *r = rv;
  for (int i = 0; i < size; ++i) {
    sprintf(r, "%s\n", symbols[i]);
    r += strlen(symbols[i]);
  }
  
  free(symbols);
#else
  char *rv = strdup("Backtrace not available on current system\n");
#endif

  return rv;
}


/** Prints trace to stderr.
 * This prints out a message trace of all messages appended to the exception
 * in chronological order starting with the oldest (first message appended
 * via constructor or append(). Output will be sent to stderr.
 */
void
Exception::print_trace() throw()
{
  messages_mutex->lock();
  fprintf(stderr,
	  "=================================================== BEGIN OF EXCEPTION =====\n");
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
	  "=================================================== END OF EXCEPTION =======\n");
  messages_mutex->unlock();
}


/** Get errno.
 * @return error number, may be 0 if not set
 */
int
Exception::get_errno() throw()
{
  return _errno;
}


/** Get primary string.
 * Messages are stored in a list. The first entry in this list is called primary
 * message. This is why it is important to have a meaningful first message!
 * @return Returns a constant char pointer with the message. The message is
 * private to the exception and may not be modified or freed (hence const)
 * If no message has been set "Unknown error" is returned. This method may be
 * overidden by other exceptions.
 * This method is also called by the runtime system if the exception was not
 * caught and resulted in a program termination.
 * @return string describing the general cause of the current error
 */
const char *
Exception::what() const throw()
{
#ifdef HAVE_EXECINFO
  print_backtrace();
#endif
  if ( messages != NULL ) {
    return messages->msg;
  } else {
    return "Unknown error";
  }
}


/** Get primary string (does not implicitly print the back trace).
 * Messages are stored in a list. The first entry in this list is called primary
 * message. This is why it is important to have a meaningful first message!
 * @return Returns a constant char pointer with the message. The message is
 * private to the exception and may not be modified or freed (hence const)
 * If no message has been set "Unknown error" is returned. This method may be
 * overidden by other exceptions.
 * This method is also called by the runtime system if the exception was not
 * caught and resulted in a program termination.
 * @return string describing the general cause of the current error
 */
const char *
Exception::what_no_backtrace() const throw()
{
  if ( messages != NULL ) {
    return messages->msg;
  } else {
    return "Unknown error";
  }
}


/** Get iterator for messages.
 * @return iterator for messages
 */
Exception::iterator
Exception::begin() throw()
{
  Exception::iterator i(messages);
  return i;
}


/** @class Exception::iterator <core/exception.h>
 * Message iterator for exceptions.
 * This iterator allows for iterating over all messages carried by an Exception.
 * @author Tim Niemueller
 */

/** Get end iterator for messages.
 * @return end iterator for messages.
 */
Exception::iterator
Exception::end() throw()
{
  Exception::iterator i;
  return i;
}


/** Constructor.
 * @param message_list list of messages, will be used unlocked so use
 * with care.
 */
Exception::iterator::iterator(message_list_t *message_list)
{
  mlist = message_list;
}


/** Plain constructor.
 * Creates a new invalid iterator (same as Exception::end()).
 */
Exception::iterator::iterator()
{
  this->mlist = NULL;
}


/** Copy constructor.
 * @param i iterator to copy
 */
Exception::iterator::iterator(const Exception::iterator & i)
{
  this->mlist = i.mlist;
}


/** Prefix ++ operator.
 * @return reference to this iterator after advancing.
 */
Exception::iterator &
Exception::iterator::operator++()
{
  if ( mlist != NULL ) {
    mlist = mlist->next;
  }
  return *this;
}


/** Postfix ++ operator.
 * @param inc used to denote postfix operator
 * @return copy of iterator before advancing.
 */
Exception::iterator
Exception::iterator::operator++(int inc)
{
  iterator i(mlist);
  if ( mlist != NULL ) {
    mlist = mlist->next;
  }
  return i;
}


/** Check equality.
 * @param i iterator to compare to
 * @return true, if iterators point to the same message, false otherwise
 */
bool
Exception::iterator::operator==(const iterator & i) const
{
  return (mlist == i.mlist);
}


/** Check inequality.
 * @param i iterator to compare to
 * @return true, if iterators point to different messages, false otherwise
 */
bool
Exception::iterator::operator!=(const iterator & i) const
{
  return (mlist != i.mlist);
}


/** Get current message.
 * Get message at current position. Returns NULL for the invalid ieterator.
 * @return message or NULL if iterator is invalid
 */
const char *
Exception::iterator::operator* () const
{
  if ( mlist != NULL ) {
    return mlist->msg;
  } else {
    return NULL;
  }
}


/** Assignment operator.
 * @param i iterator to assign to this iterator.
 * @return reference to this iterator.
 */
Exception::iterator &
Exception::iterator::operator=(const iterator &i)
{
  this->mlist = i.mlist;
  return *this;
}


} // end namespace fawkes
