/***************************************************************************
 *  utils.cpp - Common utility functions used with Golog++
 *
 *  Created: Wed 30 Oct 2019 14:31:18 CET 14:31
 *  Copyright  2019  Till Hofmann <hofmann@kbsg.rwth-aachen.de>
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

#include "utils.h"

#include <blackboard/blackboard.h>
#include <blackboard/remote.h>
#include <golog++/model/value.h>

using gologpp::Value;

namespace fawkes {
namespace gpp {

/** @class ValueToFieldVisitor
 * A visitor that converts a gologpp::Value to an interface field value.
 * The visitor checks the types of the input gologpp::Value and the ouput
 * InterfaceFieldIterator. If they match, the field is set. Otherwise, an
 * exception is thrown.
 */

/** Constructor.
 * @param field The field to set.
 * @param index The index to set if field is an array.
 */
ValueToFieldVisitor::ValueToFieldVisitor(InterfaceFieldIterator *field, unsigned int index)
: field(field), index(index)
{
}

/** Convert the given value and set the field accordingly.
 * @param v The value to set the field to.
 */
void
ValueToFieldVisitor::operator()(unsigned int v)
{
	switch (field->get_type()) {
	case IFT_INT32: field->set_int32(v, index); break;
	case IFT_INT64: field->set_int64(v, index); break;
	case IFT_UINT16: field->set_uint16(v, index); break;
	case IFT_UINT32: field->set_uint32(v, index); break;
	case IFT_UINT64: field->set_uint64(v, index); break;
	default: throw Exception("Invalid cast from unsigned int to %s", field->get_typename());
	}
}

/** Convert the given value and set the field accordingly.
 * @param v The value to set the field to.
 */
void
ValueToFieldVisitor::operator()(int v)
{
	switch (field->get_type()) {
	case IFT_INT32: field->set_int32(v, index); break;
	case IFT_INT64: field->set_int64(v, index); break;
	default: throw Exception("Invalid cast from int to %s", field->get_typename());
	}
}

/** Convert the given value and set the field accordingly.
 * @param v The value to set the field to.
 */
void
ValueToFieldVisitor::operator()(unsigned long v)
{
	switch (field->get_type()) {
	case IFT_INT64: field->set_int64(v, index); break;
	case IFT_UINT32: field->set_uint32(v, index); break;
	case IFT_UINT64: field->set_uint64(v, index); break;
	default: throw Exception("Invalid cast from unsigned long to %s", field->get_typename());
	}
}

/** Convert the given value and set the field accordingly.
 * @param v The value to set the field to.
 */
void
ValueToFieldVisitor::operator()(long v)
{
	switch (field->get_type()) {
	case IFT_UINT32: field->set_uint32(v, index); break;
	case IFT_INT64: field->set_int64(v, index); break;
	default: throw Exception("Invalid cast from long to %s", field->get_typename());
	}
}

/** Convert the given value and set the field accordingly.
 * @param v The value to set the field to.
 */
void
ValueToFieldVisitor::operator()(double v)
{
	switch (field->get_type()) {
	case IFT_FLOAT: field->set_float(v, index); break;
	case IFT_DOUBLE: field->set_double(v, index); break;
	default: throw Exception("Invalid cast from double to %s", field->get_typename());
	}
}

/** Convert the given value and set the field accordingly.
 * @param v The value to set the field to.
 */
void
ValueToFieldVisitor::operator()(std::string v)
{
	switch (field->get_type()) {
	case IFT_STRING:
		if (index != 0) {
			throw Exception("Invalid cast, string arrays are not supported");
		}
		field->set_string(v.c_str());
		break;
	case IFT_ENUM: try { field->set_enum_string(v.c_str());
		} catch (IllegalArgumentException &e) {
			throw Exception("Cannot convert string '%s' into enum: %s", v.c_str(), e.what_no_backtrace());
		}
		break;
	default: throw Exception("Invalid cast from string to %s", field->get_typename());
	}
}

/** Convert the given value and set the field accordingly.
 * @param v The value to set the field to.
 */
void
ValueToFieldVisitor::operator()(bool v)
{
	switch (field->get_type()) {
	case IFT_BOOL: field->set_bool(v, index);
	default: throw Exception("Invalid cast from bool to %s", field->get_typename());
	}
}

/** Golog++ does not support void* types. Thus, this always throws.
 * @param v The value to set the field to.
 */
void
ValueToFieldVisitor::operator()(void *v)
{
	switch (field->get_type()) {
	default: throw Exception("Invalid cast from void* to %s", field->get_typename());
	}
}

/** Not implemented yet.
 * @param v The value to set the field to.
 */
void
ValueToFieldVisitor::operator()(gologpp::CompoundType::Representation v)
{
	switch (field->get_type()) {
	default: throw Exception("Invalid cast from compound to %s", field->get_typename());
	}
}

/** Convert the given list by calling a visitor recursively for each item of the list.
 * @param v The value to set the field to.
 */
void
ValueToFieldVisitor::operator()(gologpp::ListType::Representation v)
{
	if (index != 0) {
		throw Exception("Invalid cast, cannot convert a list with an offset or nested lists");
	}
	for (size_t i = 0; i < v.size(); i++) {
		ValueToFieldVisitor visitor(field, i);
		boost::apply_visitor(visitor, v[i]->representation());
	}
}

void
value_to_field(const gologpp::Value &value, InterfaceFieldIterator *field)
{
	ValueToFieldVisitor visitor(field);
	boost::apply_visitor(visitor, value.representation());
}

Value *
field_to_value(InterfaceFieldIterator &fi, unsigned int idx)
{
	using namespace gologpp;

	switch (fi.get_type()) {
	case IFT_BOOL: return new Value(get_type<BoolType>(), fi.get_bool(idx));
	case IFT_BYTE: return new Value(get_type<NumberType>(), fi.get_byte(idx));
	case IFT_ENUM: return new Value(get_type<SymbolType>(), fi.get_enum_string(idx));
	case IFT_INT8: return new Value(get_type<NumberType>(), fi.get_int8(idx));
	case IFT_FLOAT: return new Value(get_type<NumberType>(), fi.get_float(idx));
	case IFT_INT16: return new Value(get_type<NumberType>(), fi.get_int16(idx));
	case IFT_INT32: return new Value(get_type<NumberType>(), fi.get_int32(idx));
	case IFT_INT64: return new Value(get_type<NumberType>(), fi.get_int64(idx));
	case IFT_UINT8: return new Value(get_type<NumberType>(), fi.get_uint8(idx));
	case IFT_DOUBLE: return new Value(get_type<NumberType>(), fi.get_double(idx));
	case IFT_STRING: return new Value(get_type<StringType>(), fi.get_string());
	case IFT_UINT16: return new Value(get_type<NumberType>(), fi.get_uint16(idx));
	case IFT_UINT32: return new Value(get_type<NumberType>(), fi.get_uint32(idx));
	case IFT_UINT64: return new Value(get_type<NumberType>(), fi.get_uint64(idx));
	}
	throw Exception("Unhandled interface field type");
}

/** A central point to interact with the skiller. There can only be one of these since it
 * acquires exclusive control
 * @param logger Initialized logger to use
 * @param blackboard Initialized Blackboard to use. Deleted in destructor if it's a @ref RemoteBlackBoard.
 */
SkillerManager::SkillerManager(BlackBoard *blackboard)
: BlackBoardInterfaceListener("Golog++SkillerManager"), blackboard_(blackboard), terminated_(false)
{
	skiller_iface_ = blackboard_->open_for_reading<SkillerInterface>("Skiller");
	skiller_iface_->read();
	if (!skiller_iface_->has_writer()) {
		blackboard_->close(skiller_iface_);
		delete blackboard_;
		throw Exception("No writer for Skiller interface");
	}

	bbil_add_data_interface(skiller_iface_);
	blackboard_->register_listener(this, BlackBoard::BBIL_FLAG_DATA);

	skiller_iface_->msgq_enqueue(new SkillerInterface::AcquireControlMessage());
}

/** Release exclusive control and free all resources
 */
SkillerManager::~SkillerManager()
{
	skiller_iface_->msgq_enqueue(new SkillerInterface::ReleaseControlMessage());
	blackboard_->unregister_listener(this);
	blackboard_->close(skiller_iface_);
	if (dynamic_cast<RemoteBlackBoard *>(blackboard_))
		delete blackboard_;
}

/** Start the given skillstring and call @ref end_cb when it ends
 * @param skillstring Skillstring to run
 * @param end_cb End callback. Status enum is either S_FINAL or S_FAILED.
 */
void
SkillerManager::start_skill(const std::string &skillstring,
                            const std::function<void(SkillerInterface::SkillStatusEnum)> &end_cb)
{
	std::lock_guard<std::recursive_mutex> l(mutex_);

	if (terminated_)
		return;

	skiller_iface_->read();
	if (end_callback_) {
		throw IllegalArgumentException("Cannot execute \"%s\" while \"%s\" is still running",
		                               skillstring.c_str(),
		                               skiller_iface_->skill_string());
	}

	skiller_iface_->msgq_enqueue(new SkillerInterface::ExecSkillMessage(skillstring.c_str()));
	end_callback_ = end_cb;
}

/** Stop execution of current skill (if any)
 */
void SkillerManager::stop_skill()
{
	std::lock_guard<std::recursive_mutex> l(mutex_);

	if (terminated_)
		return;

	if (end_callback_)
		skiller_iface_->msgq_enqueue(new SkillerInterface::StopExecMessage());

	end_callback_ = nullptr;
}

void
SkillerManager::bb_interface_data_refreshed(Interface *iface) throw()
{
	std::lock_guard<std::recursive_mutex> l(mutex_);

	if (terminated_ || !end_callback_)
		return;

	SkillerInterface *skiller_if = dynamic_cast<SkillerInterface *>(iface);
	if (!skiller_if) {
		return;
	}

	skiller_if->read();
	if (skiller_if->status() == SkillerInterface::SkillStatusEnum::S_FINAL
	    || skiller_if->status() == SkillerInterface::SkillStatusEnum::S_FAILED) {
		end_callback_(skiller_if->status());
		end_callback_ = nullptr;
	}
}

/** Prepare for shutdown by stopping skill execution.
 */
void SkillerManager::terminate()
{
	std::lock_guard<std::recursive_mutex> l(mutex_);

	stop_skill();
	terminated_ = true;
}

constexpr const char *
SkillerManager::name() const
{
	return "Golog++SkillerManager";
}

} // namespace gpp
} // namespace fawkes
