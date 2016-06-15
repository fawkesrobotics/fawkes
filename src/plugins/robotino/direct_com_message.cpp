/***************************************************************************
 *  direct_com_message.cpp - Message for RobotinoDirectThread
 *
 *  Created: Mon Apr 04 15:48:52 2016
 *  Copyright  2011-2016  Tim Niemueller [www.niemueller.de]
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

#include "direct_com_message.h"

#include <core/exception.h>
#include <iomanip>
#include <cstdlib>

using namespace fawkes;

/// @cond INTERNAL
const unsigned char DirectRobotinoComMessage::MSG_HEAD        = 0xAA;
const unsigned char DirectRobotinoComMessage::MSG_DATA_ESCAPE = 0x55;
const unsigned char DirectRobotinoComMessage::MSG_DATA_MANGLE = 0x20;

// 5: 0xAA + payload_size/2 ... + checksum/2
const unsigned int DirectRobotinoComMessage::MSG_METADATA_SIZE = 5;
/// @endcond INTERNAL

/** @class DirectRobotinoComMessage::ChecksumError "direct_com_message.h"
 * Excpetion thrown on checksum errors.
 */

/** Constructor.
 * @param expected expected checksum
 * @param received actually received checksm
 * @param byte1 First byte of actually received checksum
 * @param byte2 Second byte of actually received checksum
 */
DirectRobotinoComMessage::ChecksumError::ChecksumError(unsigned int expected, unsigned int received,
                                                       unsigned char byte1, unsigned char byte2)
	: Exception("Checksum verification error for Robotino message, "
	            "expected %u, got %u (%02x %02x)", expected, received, byte1, byte2)
{
}

/** @class DirectRobotinoComMessage "direct_com_message.h"
 * Robotino communication message.
 *
 * This object is used to create messages to send and parse messages
 * to read. It is designed to be generic, i.e., it provides methods to
 * add messages and its fields and to iterate over commands and read
 * fields. These methods must be called in proper sequence, no command
 * type specific processing is performed within the class. This
 * approach was chosen since we do not strive for a user-facing
 * generic transport, but rather for a minimal method to interact with
 * Robotino's microcontroller.
 *
 * A message strictly differentiates between a reading and a writing
 * mode, depending on the constructor with which it was created. The
 * reading mode is used to parse received messages and provide access
 * to its commands, the writing mode to construct messages to send.
 * Furthermore, the message assumes quick run-throughs, i.e., after
 * requesting the buffer of a writing message once, it is fixed and
 * will not be re-generated after further additions.
 *
 * Terminology is a mix in part due to the naming in OpenRobotino:
 * - Message: our representation of a message to send
 * - Command: a command field within a message, this is called tag
 *            and also command in OpenRobotino. We chose the latter.
 *
 * @author Tim Niemueller
 */

/** Constructor.
 * Create empty message for writing.
 */
DirectRobotinoComMessage::DirectRobotinoComMessage()
{
	ctor();
}


/** Constructor for initial command.
 * Create message for writing and add command for given message ID.
 * @param cmdid message ID of command to add
 */
DirectRobotinoComMessage::DirectRobotinoComMessage(command_id_t cmdid)
{
	ctor();

	add_command(cmdid);
}

/** Constructor for incoming message.
 * Create message for reading from incoming buffer.
 * @param msg the message of \p msg_size is expected to be escaped and to range from
 * the including 0xAA head byte to the checksum.
 * @param msg_size size of \p msg buffer
 */
DirectRobotinoComMessage::DirectRobotinoComMessage(const unsigned char *msg, size_t msg_size)
{
	ctor();

	mode_ = READ;

	escaped_data_ = (unsigned char *)malloc(msg_size);
	memcpy(escaped_data_, msg, msg_size);
	escaped_data_size_ = msg_size;
	size_t escaped_consumed = unescape_data();

	if (escaped_consumed < msg_size) {
		escaped_data_ = (unsigned char *)realloc(escaped_data_, escaped_consumed);
		escaped_data_size_ = escaped_consumed;
	}

	check_checksum();
}

void
DirectRobotinoComMessage::ctor()
{
	payload_size_ = 0;
	// always allocate 128 bytes, increase if necessary
	data_size_ = 128;
	data_ = (unsigned char *)malloc(data_size_);
	memset(data_, 0, data_size_);
	data_[0] = MSG_HEAD;
	cur_data_ = data_ + 3;
	cur_cmd_ = NULL;

	escaped_data_ = NULL;

	mode_ = WRITE;
}

/** Destructor. */
DirectRobotinoComMessage::~DirectRobotinoComMessage()
{
	::free(data_);
	data_size_ = payload_size_ = 0;
	if (escaped_data_)  ::free(escaped_data_);
	cur_data_ = NULL;
}


/** Assert a given message mode.
 * @param mode mode
 * @throw Exception on mode mismatch
 */
void
DirectRobotinoComMessage::assert_mode(mode_t mode) const
{
	if (mode_ == WRITE && mode == READ) {
		throw Exception("Message mode is writing, but requested reading operation");
	} else if (mode_ == READ && mode == WRITE) {
		throw Exception("Message mode is reading, but requested writing operation");
	}
}


/** Assert that a command has been selected.
 * @throw Exception if no command opened
 */
void
DirectRobotinoComMessage::assert_command() const
{
	if (! cur_cmd_) {
		throw Exception("No command has been opened for reading (call next_command)");
	}
}

/** Assert minimum available data size.
 * @param size number of bytes yet to be read
 * @throw Exception if less than \p size bytes are available
 */
void
DirectRobotinoComMessage::assert_command_data(uint8_t size) const
{
	if (payload_size_ < size || cur_data_ + size > cur_cmd_ + cur_cmd_[1] + 2) {
		throw Exception("Cannot read beyond command length %x %x (%x + %u >= %x + %u + 2)",
		                cur_data_ + size, cur_cmd_ + cur_cmd_[1] + 2, cur_data_, size, cur_cmd_, cur_cmd_[1]);
	}
}

/** Increase payload by a number of bytes.
 * This may reallocate the memory to hold the data if it exceeds the current size.
 * @param count number of bytes to extend payload by
 */
void
DirectRobotinoComMessage::inc_payload_by(uint16_t count)
{
	assert_mode(WRITE);
	if (! cur_cmd_) {
		throw Exception("Must add command before values");
	}

	if (payload_size_ + count >= data_size_ - MSG_METADATA_SIZE) {
		// need to realloc for more data
		data_ = (unsigned char *)realloc(data_, data_size_ + 128);
	}
	payload_size_ += count;
	cur_cmd_[1] += count;
}


/** Add a command header.
 * This only allocates the header. You must call the appropriate methods to
 * add the required data fields afterwards or the message will be
 * rejected/ignored by the Robotino.
 * @param cmdid command ID to add.
 */
void
DirectRobotinoComMessage::add_command(command_id_t cmdid)
{
	cur_cmd_ = cur_data_;
	cur_data_ += 2;
	inc_payload_by(2);

	cur_cmd_[0] = 0xff & cmdid;
	cur_cmd_[1] = 0;
}

/** Add 8-bit signed integer to current command.
 * @param value value to add
 * @throw Exception thrown if no command has been added, yet
 */
void
DirectRobotinoComMessage::add_int8(int8_t value)
{
	inc_payload_by(1);
	*(cur_data_++) = 0xFF & value;
}

/** Add 8-bit unsigned integer to current command.
 * @param value value to add
 * @throw Exception thrown if no command has been added, yet
 */
void
DirectRobotinoComMessage::add_uint8(uint8_t value)
{
	inc_payload_by(1);
	*(cur_data_++) = 0xFF & value;
}

/** Add 16-bit signed integer to current command.
 * @param value value to add
 * @throw Exception thrown if no command has been added, yet
 */
void
DirectRobotinoComMessage::add_int16(int16_t value)
{
	inc_payload_by(2);
	*(cur_data_++) = 0xFF & value;
	*(cur_data_++) = value >> 8;
}

/** Add 16-bit unsigned integer to current command.
 * @param value value to add
 * @throw Exception thrown if no command has been added, yet
 */
void
DirectRobotinoComMessage::add_uint16(uint16_t value)
{
	inc_payload_by(2);
	*(cur_data_++) = 0xFF & value;
	*(cur_data_++) = value >> 8;
}

/** Add 32-bit signed integer to current command.
 * @param value value to add
 * @throw Exception thrown if no command has been added, yet
 */
void
DirectRobotinoComMessage::add_int32(int32_t value)
{
	inc_payload_by(4);
	*(cur_data_++) = 0xFF & value;
	*(cur_data_++) = 0xFF & ( value >> 8 );
	*(cur_data_++) = 0xFF & ( value >> 16 );
	*(cur_data_++) = 0xFF & ( value >> 24 );
}

/** Add 32-bit unsigned integer to current command.
 * @param value value to add
 * @throw Exception thrown if no command has been added, yet
 */
void
DirectRobotinoComMessage::add_uint32(uint32_t value)
{
	inc_payload_by(4);
	*(cur_data_++) = 0xFF & value;
	*(cur_data_++) = 0xFF & ( value >> 8 );
	*(cur_data_++) = 0xFF & ( value >> 16 );
	*(cur_data_++) = 0xFF & ( value >> 24 );	
}

/** Add float to current command.
 * @param value value to add
 * @throw Exception thrown if no command has been added, yet
 */
void
DirectRobotinoComMessage::add_float(float value)
{
	inc_payload_by(4);
	const char* p = reinterpret_cast<const char*>( &value );

	for(int i = 0; i < 4; ++i) {
		*(cur_data_++) = *(p++);
	}
}

/** Rewind to read again from start.
 * @throw Exception thrown if no not a reading message
 */
void
DirectRobotinoComMessage::rewind()
{
	assert_mode(READ);
	cur_data_ = data_ + 3;
	cur_cmd_ = NULL;
}

/** Get next available command.
 * @return ID of next command, or CMDID_NONE if no more commands available
 */
DirectRobotinoComMessage::command_id_t
DirectRobotinoComMessage::next_command()
{
	assert_mode(READ);
	if (cur_cmd_ == NULL && payload_size_ >= 2) {
		// no command set but payload that could hold one
		cur_cmd_ = &data_[3];
		cur_data_ = cur_cmd_ + 2;
		return (command_id_t)cur_cmd_[0];
	} else if (cur_cmd_ && ((data_ + payload_size_ + MSG_METADATA_SIZE - 2) - (cur_cmd_ + cur_cmd_[1] + 2)) >= 2) {
		// we have a command and it does not extend beyond the payload, -2: subtract length of checksum
		cur_cmd_ += cur_cmd_[1] + 2;
		cur_data_ = cur_cmd_ + 2;
		return (command_id_t)cur_cmd_[0];
	} else {
		return CMDID_NONE;
	}
}


/** Get length of current command.
 * @return length in bytes
 */
uint8_t
DirectRobotinoComMessage::command_length() const
{
	assert_mode(READ);
	assert_command();
	return cur_cmd_[1];
}


/** Get ID of current command.
 * @return command ID
 */
DirectRobotinoComMessage::command_id_t
DirectRobotinoComMessage::command_id() const
{
	assert_mode(READ);
	assert_command();
	return (command_id_t)cur_cmd_[0];
}

/** Get 8-bit signed integer from current command.
 * This also forwards the command-internal pointer appropriately.
 * @return value
 * @throw Exception thrown if not enough data remains to be read
 */
int8_t
DirectRobotinoComMessage::get_int8()
{
	assert_mode(READ);
	assert_command();
	assert_command_data(1);

	int8_t value = (int8_t)cur_data_[0];
	cur_data_ += 1;
	return value;
}

/** Get 8-bit unsigned integer from current command.
 * This also forwards the command-internal pointer appropriately.
 * @return value
 * @throw Exception thrown if not enough data remains to be read
 */
uint8_t
DirectRobotinoComMessage::get_uint8()
{
	assert_mode(READ);
	assert_command();
	assert_command_data(1);

	uint8_t value = (uint8_t)cur_data_[0];
	cur_data_ += 1;
	return value;
}

/** Get 16-bit signed integer from current command.
 * This also forwards the command-internal pointer appropriately.
 * @return value
 * @throw Exception thrown if not enough data remains to be read
 */
int16_t
DirectRobotinoComMessage::get_int16()
{
	assert_mode(READ);
	assert_command();
	assert_command_data(2);

	int16_t value = (uint8_t)cur_data_[0];
	value |= ( (int16_t)cur_data_[1] << 8 );
	cur_data_ += 2;
	return value;
}

/** Get 16-bit unsigned integer from current command.
 * This also forwards the command-internal pointer appropriately.
 * @return value
 * @throw Exception thrown if not enough data remains to be read
 */
uint16_t
DirectRobotinoComMessage::get_uint16()
{
	assert_mode(READ);
	assert_command();
	assert_command_data(2);

	uint16_t value = (uint8_t)cur_data_[0];
	uint16_t h = (uint8_t)cur_data_[1];
	value |= (h << 8);
	cur_data_ += 2;
	return value;
}

/** Parse 16-bit unsigned integer from given buffer.
 * @param buf buffer at least of size 2 bytes
 * @return value
 * @throw Exception thrown if not enough data remains to be read
 */
uint16_t
DirectRobotinoComMessage::parse_uint16(const unsigned char *buf)
{
	uint16_t value = (uint8_t)buf[0];
	uint16_t h = (uint8_t)buf[1];
	value |= (h << 8);
	return value;
}

/** Get 32-bit signed integer from current command.
 * This also forwards the command-internal pointer appropriately.
 * @return value
 * @throw Exception thrown if not enough data remains to be read
 */
int32_t
DirectRobotinoComMessage::get_int32()
{
	assert_mode(READ);
	assert_command();
	assert_command_data(4);

	int32_t value = (uint8_t)cur_data_[0];
	int32_t h1 = (uint8_t)cur_data_[1];
	int32_t h2 = (uint8_t)cur_data_[2];
	int32_t h3 = (uint8_t)cur_data_[3];
	value |= (h1 << 8);
	value |= (h2 << 16);
	value |= (h3 << 24);
	cur_data_ += 4;
	return value;
}

/** Get 32-bit unsigned integer from current command.
 * This also forwards the command-internal pointer appropriately.
 * @return value
 * @throw Exception thrown if not enough data remains to be read
 */
uint32_t
DirectRobotinoComMessage::get_uint32()
{
	assert_mode(READ);
	assert_command();
	assert_command_data(4);

	uint32_t value = (uint8_t)cur_data_[0];
	uint32_t h1 = (uint8_t)cur_data_[1];
	uint32_t h2 = (uint8_t)cur_data_[2];
	uint32_t h3 = (uint8_t)cur_data_[3];
	value |= (h1 << 8);
	value |= (h2 << 16);
	value |= (h3 << 24);
	cur_data_ += 4;
	return value;
}

/** Get float from current command.
 * This also forwards the command-internal pointer appropriately.
 * @return value
 * @throw Exception thrown if not enough data remains to be read
 */
float
DirectRobotinoComMessage::get_float()
{
	assert_mode(READ);
	assert_command();
	assert_command_data(4);

	float value;
	char* p = reinterpret_cast<char*>( &value );
	*(p++) = cur_data_[0];
	*(p++) = cur_data_[1];
	*(p++) = cur_data_[2];
	*(p++) = cur_data_[3];
	cur_data_ += 4;
	return value;
}

/** Get string from current command.
 * This consumes all remaining data in the current command.
 * @return value
 * @throw Exception thrown if no data remains to be read
 */
std::string
DirectRobotinoComMessage::get_string()
{
	assert_mode(READ);
	assert_command();
	assert_command_data(1);

	size_t remaining = (cur_cmd_ + cur_cmd_[1] + 2) - cur_data_;
	std::string value((const char *)cur_data_, remaining);
	cur_data_ += remaining;
	return value;
}


/** Skip 8-bit signed integer from current command.
 * This also forwards the command-internal pointer appropriately.
 * @throw Exception thrown if not enough data remains to be read
 */
void
DirectRobotinoComMessage::skip_int8()
{
	assert_mode(READ);
	assert_command();
	assert_command_data(1);

	cur_data_ += 1;
}

/** Skip 8-bit unsigned integer from current command.
 * This also forwards the command-internal pointer appropriately.
 * @throw Exception thrown if not enough data remains to be read
 */
void
DirectRobotinoComMessage::skip_uint8()
{
	assert_mode(READ);
	assert_command();
	assert_command_data(1);

	cur_data_ += 1;
}

/** Skip 16-bit signed integer from current command.
 * This also forwards the command-internal pointer appropriately.
 * @throw Exception thrown if not enough data remains to be read
 */
void
DirectRobotinoComMessage::skip_int16()
{
	assert_mode(READ);
	assert_command();
	assert_command_data(2);

	cur_data_ += 2;
}

/** Skip 16-bit unsigned integer from current command.
 * This also forwards the command-internal pointer appropriately.
 * @throw Exception thrown if not enough data remains to be read
 */
void
DirectRobotinoComMessage::skip_uint16()
{
	assert_mode(READ);
	assert_command();
	assert_command_data(2);

	cur_data_ += 2;
}

/** Skip 32-bit signed integer from current command.
 * This also forwards the command-internal pointer appropriately.
 * @throw Exception thrown if not enough data remains to be read
 */
void
DirectRobotinoComMessage::skip_int32()
{
	assert_mode(READ);
	assert_command();
	assert_command_data(4);

	cur_data_ += 4;
}

/** Skip 32-bit unsigned integer from current command.
 * This also forwards the command-internal pointer appropriately.
 * @throw Exception thrown if not enough data remains to be read
 */
void
DirectRobotinoComMessage::skip_uint32()
{
	assert_mode(READ);
	assert_command();
	assert_command_data(4);

	cur_data_ += 4;
}

/** Skip float from current command.
 * This also forwards the command-internal pointer appropriately.
 * @throw Exception thrown if not enough data remains to be read
 */
void
DirectRobotinoComMessage::skip_float()
{
	assert_mode(READ);
	assert_command();
	assert_command_data(4);

	cur_data_ += 4;
}


/** Size of escaped buffer.
 * In particular, after calling the parsing ctor this denotes how many
 * bytes of the input buffer have been consumed.
 * On message creation, only provides meaningful values after calling
 * pack() or buffer().
 * @return size of escaped buffer
 */
size_t
DirectRobotinoComMessage::escaped_data_size()
{
	return escaped_data_size_;
}


/** Get payload size.
 * @return payload size
 */
size_t
DirectRobotinoComMessage::payload_size()
{
	return payload_size_;
}

/** Get internal data buffer size.
 * @return data buffer size
 */
size_t
DirectRobotinoComMessage::data_size()
{
	return data_size_;
}

/** Perform message escaping. */
void
DirectRobotinoComMessage::escape()
{
	unsigned short to_escape = 0;
	for (int i = 1; i < payload_size_ + 4; ++i) {
		if (data_[i] == MSG_HEAD || data_[i] == MSG_DATA_ESCAPE) {
			++to_escape;
		}
	}
	if (escaped_data_)  ::free(escaped_data_);
	escaped_data_size_ = payload_size_ + MSG_METADATA_SIZE + to_escape;
	escaped_data_ = (unsigned char *)malloc(escaped_data_size_);

	if (to_escape > 0) {
		escaped_data_[0] = MSG_HEAD;
		unsigned char *p = escaped_data_;
		*p++ = MSG_HEAD;
		for (unsigned int i = 1; i < payload_size_ + (MSG_METADATA_SIZE-1); ++i) {
			if (data_[i] == MSG_HEAD || data_[i] == MSG_DATA_ESCAPE) {
				*p++ = MSG_DATA_ESCAPE;
				*p++ = data_[i] ^ MSG_DATA_MANGLE;
			} else {
				*p++ = data_[i];
			}
		}
	} else {
		memcpy(escaped_data_, data_, escaped_data_size_);
	}
}

/** Unescape a number of unescaped bytes.
 * @param unescaped buffer to contain the unescaped data on return,
 * must be at least of \p unescaped_size number of bytes
 * @param unescaped_size expected number of bytes to unescape
 * from input buffer
 * @param escaped escaped buffer to process
 * @param escaped_size size of escaped_buffer
 * @return number of bytes consumed from escaped buffer
 * @throw Exception if not enough bytes could be unescaped
 */
size_t
DirectRobotinoComMessage::unescape(unsigned char *unescaped, size_t unescaped_size,
                                   const unsigned char *escaped, size_t escaped_size)
{
	if (unescaped_size == 0)  return 0;

	unsigned int j = 0;
	for (unsigned int i = 0; i < escaped_size; ++i) {
		if (escaped[i] == MSG_DATA_ESCAPE) {
			if (i >= escaped_size - 1) {
				throw Exception("Read escaped byte last in message");
			}
			unescaped[j++] = escaped[i+1] ^ MSG_DATA_MANGLE;
			i += 1;
		} else {
			unescaped[j++] = escaped[i];
		}
		if (j == unescaped_size)  return i+1;
	}

	throw Exception("Not enough escaped bytes for unescaping");
}

/** Unescape all data.
 * @return number of bytes consumed from escaped buffer
 */
size_t
DirectRobotinoComMessage::unescape_data()
{
	if (! escaped_data_ || escaped_data_size_ == 0) {
		throw Exception("No escaped data to unescape");
	}

	if (data_size_ < 3) {
		data_ = (unsigned char *)realloc(data_, 3);
		data_[0] = MSG_HEAD;
	}
	// +1: HEAD
	size_t consumed_bytes = unescape(&data_[1], 2, &escaped_data_[1], escaped_data_size_-1) + 1;
	size_t unescaped_size = parse_uint16(&data_[1]) + 2; // +2: checksum

	if (data_size_ < unescaped_size + 3) {
		data_ = (unsigned char *)realloc(data_, unescaped_size + 3); // +3: HEAD, LENGTH
		data_size_ = unescaped_size + 3;
	}
	payload_size_ = unescaped_size - 2; // -2: no checksum

	consumed_bytes +=
		unescape(&data_[3], unescaped_size,
		         &escaped_data_[consumed_bytes], escaped_data_size_ - consumed_bytes);

	return consumed_bytes;
}


/** Get access to buffer for sending.
 * This implies packing. Note that after calling this methods later
 * modifications to the message will be ignored. The buffer is invalidated
 * on the destruction of the message.
 * @return buffer of escaped data.
 */
boost::asio::const_buffer
DirectRobotinoComMessage::buffer()
{
	pack();
	return boost::asio::buffer(escaped_data_, escaped_data_size_);
}

/** Pack message.
 * This escapes the data to be sent.
 */
void
DirectRobotinoComMessage::pack()
{
	if (! escaped_data_) {
		data_[1] = 0xff & payload_size_;
		data_[2] = payload_size_ >> 8;
		unsigned short checksum_value = checksum();
		data_[payload_size_ + 3] = 0xff & checksum_value;
		data_[payload_size_ + 4] = checksum_value >> 8;
		escape();
	}
}

/** Get checksum of message.
 * @return checksum
 */
uint16_t
DirectRobotinoComMessage::checksum() const
{
	uint16_t rv = 0;
	const unsigned char* p = &data_[1];
	// -3:  do not include the 0xAA header and checksum
	for (unsigned int i = 0; i < payload_size_ + (MSG_METADATA_SIZE-3); ++i) {
		rv += (uint8_t)*p;
		++p;
	}
	return 0xffff & ( (1<<16) - rv );
}

/** Check the checksum.
 * This is for a parsed message to verify that the checksum in the received
 * messages matches the self-calculated one.
 */
void
DirectRobotinoComMessage::check_checksum() const
{
	uint16_t checksum_v = checksum();
	uint16_t packet_checksum = parse_uint16(&data_[payload_size_ + 3]);
	if (checksum_v != packet_checksum) {
		throw ChecksumError(checksum_v, packet_checksum,
		                    data_[payload_size_ + 3], data_[payload_size_ + 4]);
	}
}

/** Generate string representation of message.
 * @param escaped true to serialize escaped buffer, false to use unescaped buffer
 * @return string representation of message in hexadecimal view with parantheses
 * to indicate command boundaries.
 */
std::string
DirectRobotinoComMessage::to_string(bool escaped)
{
	boost::asio::const_buffer b;
	const unsigned char *bp;
	size_t bsize;
	if (escaped) {
		b = buffer();
		bp = boost::asio::buffer_cast<const unsigned char*>(b);
		bsize = boost::asio::buffer_size(b);
	} else {
		bp = data_;
		bsize = payload_size_ + MSG_METADATA_SIZE;
	}

	std::string rv;
	// this buffer must hold and hex representation of a byte with whitespace
	char tmp[8];

	// used for adding braces to indicate commands
	int16_t cmd_l = -1;
	int16_t cmd_i = 0;

	for (unsigned int i = 0; i < bsize; ++i) {
		bool cmd_opened = false;
		bool cmd_closed = false;
		if (! escaped) {
			if (i == 3 && bsize > MSG_METADATA_SIZE) {
				cmd_l = bp[i+1];
				cmd_i = 0;
				cmd_opened = true;
			} else if (i > 3 && cmd_l >= 0 && cmd_i == cmd_l) {
				cmd_closed = true;
				cmd_l = -1;
				cmd_i = 0;
			} else if (i > 3 && cmd_l < 0 && bsize - i > 2) {
				cmd_opened = true;
				cmd_l = bp[i+1];
				cmd_i = 0;
			} else {
				++cmd_i;
			}
		}

		if (i > 0 && (i+1) % 16 == 0) {
			snprintf(tmp, 8, "%s%02x%s\n", cmd_opened ? "(" : " ", bp[i], cmd_closed ? ")" : " ");
		} else if (i > 0 && (i+1) % 8 == 0) {
			snprintf(tmp, 8, "%s%02x%s   ", cmd_opened ? "(" : " ", bp[i], cmd_closed ? ")" : " ");
		} else {
			snprintf(tmp, 8, "%s%02x%s", cmd_opened ? "(" : " ", bp[i], cmd_closed ? ")" : " ");
		}
		rv += tmp;
	}
	if ((bsize-1) % 16 != 0) {
		rv += "\n";
	}

	return rv;
}
