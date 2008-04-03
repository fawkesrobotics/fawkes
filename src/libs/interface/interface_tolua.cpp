/*
** Lua binding: interface
** Generated automatically by tolua++-1.0.92
*/

#ifndef __cplusplus
#include "stdlib.h"
#endif
#include "string.h"

#include "tolua++.h"

/* Exported function */
TOLUA_API int  tolua_interface_open (lua_State* tolua_S);

#include <interface/interface.h>
#include <interface/message_queue.h>
#include <interface/message.h>

/* function to release collected object via destructor */
#ifdef __cplusplus

static int tolua_collect_Message (lua_State* tolua_S)
{
 Message* self = (Message*) tolua_tousertype(tolua_S,1,0);
	delete self;
	return 0;
}

static int tolua_collect_MessageQueue (lua_State* tolua_S)
{
 MessageQueue* self = (MessageQueue*) tolua_tousertype(tolua_S,1,0);
	delete self;
	return 0;
}

static int tolua_collect_MessageQueue__MessageIterator (lua_State* tolua_S)
{
 MessageQueue::MessageIterator* self = (MessageQueue::MessageIterator*) tolua_tousertype(tolua_S,1,0);
	delete self;
	return 0;
}

static int tolua_collect_size_t (lua_State* tolua_S)
{
 size_t* self = (size_t*) tolua_tousertype(tolua_S,1,0);
	delete self;
	return 0;
}
#endif


/* function to register type */
static void tolua_reg_types (lua_State* tolua_S)
{
 tolua_usertype(tolua_S,"Message");
 tolua_usertype(tolua_S,"size_t");
 tolua_usertype(tolua_S,"MessageQueue::MessageIterator");
 tolua_usertype(tolua_S,"MessageQueue");
 tolua_usertype(tolua_S,"RefCount");
 tolua_usertype(tolua_S,"Interface");
}

/* method: oftype of class  Interface */
#ifndef TOLUA_DISABLE_tolua_interface_Interface_oftype00
static int tolua_interface_Interface_oftype00(lua_State* tolua_S)
{
#ifndef TOLUA_RELEASE
 tolua_Error tolua_err;
 if (
     !tolua_isusertype(tolua_S,1,"const Interface",0,&tolua_err) ||
     !tolua_isstring(tolua_S,2,0,&tolua_err) ||
     !tolua_isnoobj(tolua_S,3,&tolua_err)
 )
  goto tolua_lerror;
 else
#endif
 {
  const Interface* self = (const Interface*)  tolua_tousertype(tolua_S,1,0);
  const char* interface_type = ((const char*)  tolua_tostring(tolua_S,2,0));
#ifndef TOLUA_RELEASE
  if (!self) tolua_error(tolua_S,"invalid 'self' in function 'oftype'",NULL);
#endif
  {
   bool tolua_ret = (bool)  self->oftype(interface_type);
   tolua_pushboolean(tolua_S,(bool)tolua_ret);
  }
 }
 return 1;
#ifndef TOLUA_RELEASE
 tolua_lerror:
 tolua_error(tolua_S,"#ferror in function 'oftype'.",&tolua_err);
 return 0;
#endif
}
#endif //#ifndef TOLUA_DISABLE

/* method: datachunk of class  Interface */
#ifndef TOLUA_DISABLE_tolua_interface_Interface_datachunk00
static int tolua_interface_Interface_datachunk00(lua_State* tolua_S)
{
#ifndef TOLUA_RELEASE
 tolua_Error tolua_err;
 if (
     !tolua_isusertype(tolua_S,1,"const Interface",0,&tolua_err) ||
     !tolua_isnoobj(tolua_S,2,&tolua_err)
 )
  goto tolua_lerror;
 else
#endif
 {
  const Interface* self = (const Interface*)  tolua_tousertype(tolua_S,1,0);
#ifndef TOLUA_RELEASE
  if (!self) tolua_error(tolua_S,"invalid 'self' in function 'datachunk'",NULL);
#endif
  {
   const void* tolua_ret = (const void*)  self->datachunk();
   tolua_pushuserdata(tolua_S,(void*)tolua_ret);
  }
 }
 return 1;
#ifndef TOLUA_RELEASE
 tolua_lerror:
 tolua_error(tolua_S,"#ferror in function 'datachunk'.",&tolua_err);
 return 0;
#endif
}
#endif //#ifndef TOLUA_DISABLE

/* method: datasize of class  Interface */
#ifndef TOLUA_DISABLE_tolua_interface_Interface_datasize00
static int tolua_interface_Interface_datasize00(lua_State* tolua_S)
{
#ifndef TOLUA_RELEASE
 tolua_Error tolua_err;
 if (
     !tolua_isusertype(tolua_S,1,"const Interface",0,&tolua_err) ||
     !tolua_isnoobj(tolua_S,2,&tolua_err)
 )
  goto tolua_lerror;
 else
#endif
 {
  const Interface* self = (const Interface*)  tolua_tousertype(tolua_S,1,0);
#ifndef TOLUA_RELEASE
  if (!self) tolua_error(tolua_S,"invalid 'self' in function 'datasize'",NULL);
#endif
  {
   unsigned int tolua_ret = (unsigned int)  self->datasize();
   tolua_pushnumber(tolua_S,(lua_Number)tolua_ret);
  }
 }
 return 1;
#ifndef TOLUA_RELEASE
 tolua_lerror:
 tolua_error(tolua_S,"#ferror in function 'datasize'.",&tolua_err);
 return 0;
#endif
}
#endif //#ifndef TOLUA_DISABLE

/* method: type of class  Interface */
#ifndef TOLUA_DISABLE_tolua_interface_Interface_type00
static int tolua_interface_Interface_type00(lua_State* tolua_S)
{
#ifndef TOLUA_RELEASE
 tolua_Error tolua_err;
 if (
     !tolua_isusertype(tolua_S,1,"const Interface",0,&tolua_err) ||
     !tolua_isnoobj(tolua_S,2,&tolua_err)
 )
  goto tolua_lerror;
 else
#endif
 {
  const Interface* self = (const Interface*)  tolua_tousertype(tolua_S,1,0);
#ifndef TOLUA_RELEASE
  if (!self) tolua_error(tolua_S,"invalid 'self' in function 'type'",NULL);
#endif
  {
   const char* tolua_ret = (const char*)  self->type();
   tolua_pushstring(tolua_S,(const char*)tolua_ret);
  }
 }
 return 1;
#ifndef TOLUA_RELEASE
 tolua_lerror:
 tolua_error(tolua_S,"#ferror in function 'type'.",&tolua_err);
 return 0;
#endif
}
#endif //#ifndef TOLUA_DISABLE

/* method: id of class  Interface */
#ifndef TOLUA_DISABLE_tolua_interface_Interface_id00
static int tolua_interface_Interface_id00(lua_State* tolua_S)
{
#ifndef TOLUA_RELEASE
 tolua_Error tolua_err;
 if (
     !tolua_isusertype(tolua_S,1,"const Interface",0,&tolua_err) ||
     !tolua_isnoobj(tolua_S,2,&tolua_err)
 )
  goto tolua_lerror;
 else
#endif
 {
  const Interface* self = (const Interface*)  tolua_tousertype(tolua_S,1,0);
#ifndef TOLUA_RELEASE
  if (!self) tolua_error(tolua_S,"invalid 'self' in function 'id'",NULL);
#endif
  {
   const char* tolua_ret = (const char*)  self->id();
   tolua_pushstring(tolua_S,(const char*)tolua_ret);
  }
 }
 return 1;
#ifndef TOLUA_RELEASE
 tolua_lerror:
 tolua_error(tolua_S,"#ferror in function 'id'.",&tolua_err);
 return 0;
#endif
}
#endif //#ifndef TOLUA_DISABLE

/* method: uid of class  Interface */
#ifndef TOLUA_DISABLE_tolua_interface_Interface_uid00
static int tolua_interface_Interface_uid00(lua_State* tolua_S)
{
#ifndef TOLUA_RELEASE
 tolua_Error tolua_err;
 if (
     !tolua_isusertype(tolua_S,1,"const Interface",0,&tolua_err) ||
     !tolua_isnoobj(tolua_S,2,&tolua_err)
 )
  goto tolua_lerror;
 else
#endif
 {
  const Interface* self = (const Interface*)  tolua_tousertype(tolua_S,1,0);
#ifndef TOLUA_RELEASE
  if (!self) tolua_error(tolua_S,"invalid 'self' in function 'uid'",NULL);
#endif
  {
   const char* tolua_ret = (const char*)  self->uid();
   tolua_pushstring(tolua_S,(const char*)tolua_ret);
  }
 }
 return 1;
#ifndef TOLUA_RELEASE
 tolua_lerror:
 tolua_error(tolua_S,"#ferror in function 'uid'.",&tolua_err);
 return 0;
#endif
}
#endif //#ifndef TOLUA_DISABLE

/* method: serial of class  Interface */
#ifndef TOLUA_DISABLE_tolua_interface_Interface_serial00
static int tolua_interface_Interface_serial00(lua_State* tolua_S)
{
#ifndef TOLUA_RELEASE
 tolua_Error tolua_err;
 if (
     !tolua_isusertype(tolua_S,1,"const Interface",0,&tolua_err) ||
     !tolua_isnoobj(tolua_S,2,&tolua_err)
 )
  goto tolua_lerror;
 else
#endif
 {
  const Interface* self = (const Interface*)  tolua_tousertype(tolua_S,1,0);
#ifndef TOLUA_RELEASE
  if (!self) tolua_error(tolua_S,"invalid 'self' in function 'serial'",NULL);
#endif
  {
   unsigned int tolua_ret = (unsigned int)  self->serial();
   tolua_pushnumber(tolua_S,(lua_Number)tolua_ret);
  }
 }
 return 1;
#ifndef TOLUA_RELEASE
 tolua_lerror:
 tolua_error(tolua_S,"#ferror in function 'serial'.",&tolua_err);
 return 0;
#endif
}
#endif //#ifndef TOLUA_DISABLE

/* method: mem_serial of class  Interface */
#ifndef TOLUA_DISABLE_tolua_interface_Interface_mem_serial00
static int tolua_interface_Interface_mem_serial00(lua_State* tolua_S)
{
#ifndef TOLUA_RELEASE
 tolua_Error tolua_err;
 if (
     !tolua_isusertype(tolua_S,1,"const Interface",0,&tolua_err) ||
     !tolua_isnoobj(tolua_S,2,&tolua_err)
 )
  goto tolua_lerror;
 else
#endif
 {
  const Interface* self = (const Interface*)  tolua_tousertype(tolua_S,1,0);
#ifndef TOLUA_RELEASE
  if (!self) tolua_error(tolua_S,"invalid 'self' in function 'mem_serial'",NULL);
#endif
  {
   unsigned int tolua_ret = (unsigned int)  self->mem_serial();
   tolua_pushnumber(tolua_S,(lua_Number)tolua_ret);
  }
 }
 return 1;
#ifndef TOLUA_RELEASE
 tolua_lerror:
 tolua_error(tolua_S,"#ferror in function 'mem_serial'.",&tolua_err);
 return 0;
#endif
}
#endif //#ifndef TOLUA_DISABLE

/* method: operator== of class  Interface */
#ifndef TOLUA_DISABLE_tolua_interface_Interface__eq00
static int tolua_interface_Interface__eq00(lua_State* tolua_S)
{
#ifndef TOLUA_RELEASE
 tolua_Error tolua_err;
 if (
     !tolua_isusertype(tolua_S,1,"const Interface",0,&tolua_err) ||
     !tolua_isusertype(tolua_S,2,"Interface",0,&tolua_err) ||
     !tolua_isnoobj(tolua_S,3,&tolua_err)
 )
  goto tolua_lerror;
 else
#endif
 {
  const Interface* self = (const Interface*)  tolua_tousertype(tolua_S,1,0);
  Interface* comp = ((Interface*)  tolua_tousertype(tolua_S,2,0));
#ifndef TOLUA_RELEASE
  if (!self) tolua_error(tolua_S,"invalid 'self' in function 'operator=='",NULL);
#endif
  {
   bool tolua_ret = (bool)  self->operator==(*comp);
   tolua_pushboolean(tolua_S,(bool)tolua_ret);
  }
 }
 return 1;
#ifndef TOLUA_RELEASE
 tolua_lerror:
 tolua_error(tolua_S,"#ferror in function '.eq'.",&tolua_err);
 return 0;
#endif
}
#endif //#ifndef TOLUA_DISABLE

/* method: hash of class  Interface */
#ifndef TOLUA_DISABLE_tolua_interface_Interface_hash00
static int tolua_interface_Interface_hash00(lua_State* tolua_S)
{
#ifndef TOLUA_RELEASE
 tolua_Error tolua_err;
 if (
     !tolua_isusertype(tolua_S,1,"const Interface",0,&tolua_err) ||
     !tolua_isnoobj(tolua_S,2,&tolua_err)
 )
  goto tolua_lerror;
 else
#endif
 {
  const Interface* self = (const Interface*)  tolua_tousertype(tolua_S,1,0);
#ifndef TOLUA_RELEASE
  if (!self) tolua_error(tolua_S,"invalid 'self' in function 'hash'",NULL);
#endif
  {
   unsigned const char* tolua_ret = ( unsigned const char*)  self->hash();
   tolua_pushstring(tolua_S,(const char*)tolua_ret);
  }
 }
 return 1;
#ifndef TOLUA_RELEASE
 tolua_lerror:
 tolua_error(tolua_S,"#ferror in function 'hash'.",&tolua_err);
 return 0;
#endif
}
#endif //#ifndef TOLUA_DISABLE

/* method: hash_size of class  Interface */
#ifndef TOLUA_DISABLE_tolua_interface_Interface_hash_size00
static int tolua_interface_Interface_hash_size00(lua_State* tolua_S)
{
#ifndef TOLUA_RELEASE
 tolua_Error tolua_err;
 if (
     !tolua_isusertype(tolua_S,1,"const Interface",0,&tolua_err) ||
     !tolua_isnoobj(tolua_S,2,&tolua_err)
 )
  goto tolua_lerror;
 else
#endif
 {
  const Interface* self = (const Interface*)  tolua_tousertype(tolua_S,1,0);
#ifndef TOLUA_RELEASE
  if (!self) tolua_error(tolua_S,"invalid 'self' in function 'hash_size'",NULL);
#endif
  {
   size_t tolua_ret = (size_t)  self->hash_size();
   {
#ifdef __cplusplus
    void* tolua_obj = new size_t(tolua_ret);
    tolua_pushusertype_and_takeownership(tolua_S,tolua_obj,"size_t");
#else
    void* tolua_obj = tolua_copy(tolua_S,(void*)&tolua_ret,sizeof(size_t));
    tolua_pushusertype_and_takeownership(tolua_S,tolua_obj,"size_t");
#endif
   }
  }
 }
 return 1;
#ifndef TOLUA_RELEASE
 tolua_lerror:
 tolua_error(tolua_S,"#ferror in function 'hash_size'.",&tolua_err);
 return 0;
#endif
}
#endif //#ifndef TOLUA_DISABLE

/* method: hash_printable of class  Interface */
#ifndef TOLUA_DISABLE_tolua_interface_Interface_hash_printable00
static int tolua_interface_Interface_hash_printable00(lua_State* tolua_S)
{
#ifndef TOLUA_RELEASE
 tolua_Error tolua_err;
 if (
     !tolua_isusertype(tolua_S,1,"const Interface",0,&tolua_err) ||
     !tolua_isnoobj(tolua_S,2,&tolua_err)
 )
  goto tolua_lerror;
 else
#endif
 {
  const Interface* self = (const Interface*)  tolua_tousertype(tolua_S,1,0);
#ifndef TOLUA_RELEASE
  if (!self) tolua_error(tolua_S,"invalid 'self' in function 'hash_printable'",NULL);
#endif
  {
   const char* tolua_ret = (const char*)  self->hash_printable();
   tolua_pushstring(tolua_S,(const char*)tolua_ret);
  }
 }
 return 1;
#ifndef TOLUA_RELEASE
 tolua_lerror:
 tolua_error(tolua_S,"#ferror in function 'hash_printable'.",&tolua_err);
 return 0;
#endif
}
#endif //#ifndef TOLUA_DISABLE

/* method: is_writer of class  Interface */
#ifndef TOLUA_DISABLE_tolua_interface_Interface_is_writer00
static int tolua_interface_Interface_is_writer00(lua_State* tolua_S)
{
#ifndef TOLUA_RELEASE
 tolua_Error tolua_err;
 if (
     !tolua_isusertype(tolua_S,1,"const Interface",0,&tolua_err) ||
     !tolua_isnoobj(tolua_S,2,&tolua_err)
 )
  goto tolua_lerror;
 else
#endif
 {
  const Interface* self = (const Interface*)  tolua_tousertype(tolua_S,1,0);
#ifndef TOLUA_RELEASE
  if (!self) tolua_error(tolua_S,"invalid 'self' in function 'is_writer'",NULL);
#endif
  {
   bool tolua_ret = (bool)  self->is_writer();
   tolua_pushboolean(tolua_S,(bool)tolua_ret);
  }
 }
 return 1;
#ifndef TOLUA_RELEASE
 tolua_lerror:
 tolua_error(tolua_S,"#ferror in function 'is_writer'.",&tolua_err);
 return 0;
#endif
}
#endif //#ifndef TOLUA_DISABLE

/* method: set_from_chunk of class  Interface */
#ifndef TOLUA_DISABLE_tolua_interface_Interface_set_from_chunk00
static int tolua_interface_Interface_set_from_chunk00(lua_State* tolua_S)
{
#ifndef TOLUA_RELEASE
 tolua_Error tolua_err;
 if (
     !tolua_isusertype(tolua_S,1,"Interface",0,&tolua_err) ||
     !tolua_isuserdata(tolua_S,2,0,&tolua_err) ||
     !tolua_isnoobj(tolua_S,3,&tolua_err)
 )
  goto tolua_lerror;
 else
#endif
 {
  Interface* self = (Interface*)  tolua_tousertype(tolua_S,1,0);
  void* chunk = ((void*)  tolua_touserdata(tolua_S,2,0));
#ifndef TOLUA_RELEASE
  if (!self) tolua_error(tolua_S,"invalid 'self' in function 'set_from_chunk'",NULL);
#endif
  {
   self->set_from_chunk(chunk);
  }
 }
 return 0;
#ifndef TOLUA_RELEASE
 tolua_lerror:
 tolua_error(tolua_S,"#ferror in function 'set_from_chunk'.",&tolua_err);
 return 0;
#endif
}
#endif //#ifndef TOLUA_DISABLE

/* method: create_message of class  Interface */
#ifndef TOLUA_DISABLE_tolua_interface_Interface_create_message00
static int tolua_interface_Interface_create_message00(lua_State* tolua_S)
{
#ifndef TOLUA_RELEASE
 tolua_Error tolua_err;
 if (
     !tolua_isusertype(tolua_S,1,"const Interface",0,&tolua_err) ||
     !tolua_isstring(tolua_S,2,0,&tolua_err) ||
     !tolua_isnoobj(tolua_S,3,&tolua_err)
 )
  goto tolua_lerror;
 else
#endif
 {
  const Interface* self = (const Interface*)  tolua_tousertype(tolua_S,1,0);
  const char* type = ((const char*)  tolua_tostring(tolua_S,2,0));
#ifndef TOLUA_RELEASE
  if (!self) tolua_error(tolua_S,"invalid 'self' in function 'create_message'",NULL);
#endif
  {
   Message* tolua_ret = (Message*)  self->create_message(type);
   tolua_pushusertype(tolua_S,(void*)tolua_ret,"Message");
  }
 }
 return 1;
#ifndef TOLUA_RELEASE
 tolua_lerror:
 tolua_error(tolua_S,"#ferror in function 'create_message'.",&tolua_err);
 return 0;
#endif
}
#endif //#ifndef TOLUA_DISABLE

/* method: read of class  Interface */
#ifndef TOLUA_DISABLE_tolua_interface_Interface_read00
static int tolua_interface_Interface_read00(lua_State* tolua_S)
{
#ifndef TOLUA_RELEASE
 tolua_Error tolua_err;
 if (
     !tolua_isusertype(tolua_S,1,"Interface",0,&tolua_err) ||
     !tolua_isnoobj(tolua_S,2,&tolua_err)
 )
  goto tolua_lerror;
 else
#endif
 {
  Interface* self = (Interface*)  tolua_tousertype(tolua_S,1,0);
#ifndef TOLUA_RELEASE
  if (!self) tolua_error(tolua_S,"invalid 'self' in function 'read'",NULL);
#endif
  {
   self->read();
  }
 }
 return 0;
#ifndef TOLUA_RELEASE
 tolua_lerror:
 tolua_error(tolua_S,"#ferror in function 'read'.",&tolua_err);
 return 0;
#endif
}
#endif //#ifndef TOLUA_DISABLE

/* method: write of class  Interface */
#ifndef TOLUA_DISABLE_tolua_interface_Interface_write00
static int tolua_interface_Interface_write00(lua_State* tolua_S)
{
#ifndef TOLUA_RELEASE
 tolua_Error tolua_err;
 if (
     !tolua_isusertype(tolua_S,1,"Interface",0,&tolua_err) ||
     !tolua_isnoobj(tolua_S,2,&tolua_err)
 )
  goto tolua_lerror;
 else
#endif
 {
  Interface* self = (Interface*)  tolua_tousertype(tolua_S,1,0);
#ifndef TOLUA_RELEASE
  if (!self) tolua_error(tolua_S,"invalid 'self' in function 'write'",NULL);
#endif
  {
   self->write();
  }
 }
 return 0;
#ifndef TOLUA_RELEASE
 tolua_lerror:
 tolua_error(tolua_S,"#ferror in function 'write'.",&tolua_err);
 return 0;
#endif
}
#endif //#ifndef TOLUA_DISABLE

/* method: has_writer of class  Interface */
#ifndef TOLUA_DISABLE_tolua_interface_Interface_has_writer00
static int tolua_interface_Interface_has_writer00(lua_State* tolua_S)
{
#ifndef TOLUA_RELEASE
 tolua_Error tolua_err;
 if (
     !tolua_isusertype(tolua_S,1,"const Interface",0,&tolua_err) ||
     !tolua_isnoobj(tolua_S,2,&tolua_err)
 )
  goto tolua_lerror;
 else
#endif
 {
  const Interface* self = (const Interface*)  tolua_tousertype(tolua_S,1,0);
#ifndef TOLUA_RELEASE
  if (!self) tolua_error(tolua_S,"invalid 'self' in function 'has_writer'",NULL);
#endif
  {
   bool tolua_ret = (bool)  self->has_writer();
   tolua_pushboolean(tolua_S,(bool)tolua_ret);
  }
 }
 return 1;
#ifndef TOLUA_RELEASE
 tolua_lerror:
 tolua_error(tolua_S,"#ferror in function 'has_writer'.",&tolua_err);
 return 0;
#endif
}
#endif //#ifndef TOLUA_DISABLE

/* method: num_readers of class  Interface */
#ifndef TOLUA_DISABLE_tolua_interface_Interface_num_readers00
static int tolua_interface_Interface_num_readers00(lua_State* tolua_S)
{
#ifndef TOLUA_RELEASE
 tolua_Error tolua_err;
 if (
     !tolua_isusertype(tolua_S,1,"const Interface",0,&tolua_err) ||
     !tolua_isnoobj(tolua_S,2,&tolua_err)
 )
  goto tolua_lerror;
 else
#endif
 {
  const Interface* self = (const Interface*)  tolua_tousertype(tolua_S,1,0);
#ifndef TOLUA_RELEASE
  if (!self) tolua_error(tolua_S,"invalid 'self' in function 'num_readers'",NULL);
#endif
  {
   unsigned int tolua_ret = (unsigned int)  self->num_readers();
   tolua_pushnumber(tolua_S,(lua_Number)tolua_ret);
  }
 }
 return 1;
#ifndef TOLUA_RELEASE
 tolua_lerror:
 tolua_error(tolua_S,"#ferror in function 'num_readers'.",&tolua_err);
 return 0;
#endif
}
#endif //#ifndef TOLUA_DISABLE

/* method: msgq_enqueue_copy of class  Interface */
#ifndef TOLUA_DISABLE_tolua_interface_Interface_msgq_enqueue_copy00
static int tolua_interface_Interface_msgq_enqueue_copy00(lua_State* tolua_S)
{
#ifndef TOLUA_RELEASE
 tolua_Error tolua_err;
 if (
     !tolua_isusertype(tolua_S,1,"Interface",0,&tolua_err) ||
     !tolua_isusertype(tolua_S,2,"Message",0,&tolua_err) ||
     !tolua_isnoobj(tolua_S,3,&tolua_err)
 )
  goto tolua_lerror;
 else
#endif
 {
  Interface* self = (Interface*)  tolua_tousertype(tolua_S,1,0);
  Message* message = ((Message*)  tolua_tousertype(tolua_S,2,0));
#ifndef TOLUA_RELEASE
  if (!self) tolua_error(tolua_S,"invalid 'self' in function 'msgq_enqueue_copy'",NULL);
#endif
  {
   unsigned int tolua_ret = (unsigned int)  self->msgq_enqueue_copy(message);
   tolua_pushnumber(tolua_S,(lua_Number)tolua_ret);
  }
 }
 return 1;
#ifndef TOLUA_RELEASE
 tolua_lerror:
 tolua_error(tolua_S,"#ferror in function 'msgq_enqueue_copy'.",&tolua_err);
 return 0;
#endif
}
#endif //#ifndef TOLUA_DISABLE

/* method: msgq_remove of class  Interface */
#ifndef TOLUA_DISABLE_tolua_interface_Interface_msgq_remove00
static int tolua_interface_Interface_msgq_remove00(lua_State* tolua_S)
{
#ifndef TOLUA_RELEASE
 tolua_Error tolua_err;
 if (
     !tolua_isusertype(tolua_S,1,"Interface",0,&tolua_err) ||
     !tolua_isusertype(tolua_S,2,"Message",0,&tolua_err) ||
     !tolua_isnoobj(tolua_S,3,&tolua_err)
 )
  goto tolua_lerror;
 else
#endif
 {
  Interface* self = (Interface*)  tolua_tousertype(tolua_S,1,0);
  Message* message = ((Message*)  tolua_tousertype(tolua_S,2,0));
#ifndef TOLUA_RELEASE
  if (!self) tolua_error(tolua_S,"invalid 'self' in function 'msgq_remove'",NULL);
#endif
  {
   self->msgq_remove(message);
  }
 }
 return 0;
#ifndef TOLUA_RELEASE
 tolua_lerror:
 tolua_error(tolua_S,"#ferror in function 'msgq_remove'.",&tolua_err);
 return 0;
#endif
}
#endif //#ifndef TOLUA_DISABLE

/* method: msgq_remove of class  Interface */
#ifndef TOLUA_DISABLE_tolua_interface_Interface_msgq_remove01
static int tolua_interface_Interface_msgq_remove01(lua_State* tolua_S)
{
 tolua_Error tolua_err;
 if (
     !tolua_isusertype(tolua_S,1,"Interface",0,&tolua_err) ||
     !tolua_isnumber(tolua_S,2,0,&tolua_err) ||
     !tolua_isnoobj(tolua_S,3,&tolua_err)
 )
  goto tolua_lerror;
 else
 {
  Interface* self = (Interface*)  tolua_tousertype(tolua_S,1,0);
  unsigned int message_id = ((unsigned int)  tolua_tonumber(tolua_S,2,0));
#ifndef TOLUA_RELEASE
  if (!self) tolua_error(tolua_S,"invalid 'self' in function 'msgq_remove'",NULL);
#endif
  {
   self->msgq_remove(message_id);
  }
 }
 return 0;
tolua_lerror:
 return tolua_interface_Interface_msgq_remove00(tolua_S);
}
#endif //#ifndef TOLUA_DISABLE

/* method: msgq_size of class  Interface */
#ifndef TOLUA_DISABLE_tolua_interface_Interface_msgq_size00
static int tolua_interface_Interface_msgq_size00(lua_State* tolua_S)
{
#ifndef TOLUA_RELEASE
 tolua_Error tolua_err;
 if (
     !tolua_isusertype(tolua_S,1,"Interface",0,&tolua_err) ||
     !tolua_isnoobj(tolua_S,2,&tolua_err)
 )
  goto tolua_lerror;
 else
#endif
 {
  Interface* self = (Interface*)  tolua_tousertype(tolua_S,1,0);
#ifndef TOLUA_RELEASE
  if (!self) tolua_error(tolua_S,"invalid 'self' in function 'msgq_size'",NULL);
#endif
  {
   unsigned int tolua_ret = (unsigned int)  self->msgq_size();
   tolua_pushnumber(tolua_S,(lua_Number)tolua_ret);
  }
 }
 return 1;
#ifndef TOLUA_RELEASE
 tolua_lerror:
 tolua_error(tolua_S,"#ferror in function 'msgq_size'.",&tolua_err);
 return 0;
#endif
}
#endif //#ifndef TOLUA_DISABLE

/* method: msgq_flush of class  Interface */
#ifndef TOLUA_DISABLE_tolua_interface_Interface_msgq_flush00
static int tolua_interface_Interface_msgq_flush00(lua_State* tolua_S)
{
#ifndef TOLUA_RELEASE
 tolua_Error tolua_err;
 if (
     !tolua_isusertype(tolua_S,1,"Interface",0,&tolua_err) ||
     !tolua_isnoobj(tolua_S,2,&tolua_err)
 )
  goto tolua_lerror;
 else
#endif
 {
  Interface* self = (Interface*)  tolua_tousertype(tolua_S,1,0);
#ifndef TOLUA_RELEASE
  if (!self) tolua_error(tolua_S,"invalid 'self' in function 'msgq_flush'",NULL);
#endif
  {
   self->msgq_flush();
  }
 }
 return 0;
#ifndef TOLUA_RELEASE
 tolua_lerror:
 tolua_error(tolua_S,"#ferror in function 'msgq_flush'.",&tolua_err);
 return 0;
#endif
}
#endif //#ifndef TOLUA_DISABLE

/* method: msgq_lock of class  Interface */
#ifndef TOLUA_DISABLE_tolua_interface_Interface_msgq_lock00
static int tolua_interface_Interface_msgq_lock00(lua_State* tolua_S)
{
#ifndef TOLUA_RELEASE
 tolua_Error tolua_err;
 if (
     !tolua_isusertype(tolua_S,1,"Interface",0,&tolua_err) ||
     !tolua_isnoobj(tolua_S,2,&tolua_err)
 )
  goto tolua_lerror;
 else
#endif
 {
  Interface* self = (Interface*)  tolua_tousertype(tolua_S,1,0);
#ifndef TOLUA_RELEASE
  if (!self) tolua_error(tolua_S,"invalid 'self' in function 'msgq_lock'",NULL);
#endif
  {
   self->msgq_lock();
  }
 }
 return 0;
#ifndef TOLUA_RELEASE
 tolua_lerror:
 tolua_error(tolua_S,"#ferror in function 'msgq_lock'.",&tolua_err);
 return 0;
#endif
}
#endif //#ifndef TOLUA_DISABLE

/* method: msgq_try_lock of class  Interface */
#ifndef TOLUA_DISABLE_tolua_interface_Interface_msgq_try_lock00
static int tolua_interface_Interface_msgq_try_lock00(lua_State* tolua_S)
{
#ifndef TOLUA_RELEASE
 tolua_Error tolua_err;
 if (
     !tolua_isusertype(tolua_S,1,"Interface",0,&tolua_err) ||
     !tolua_isnoobj(tolua_S,2,&tolua_err)
 )
  goto tolua_lerror;
 else
#endif
 {
  Interface* self = (Interface*)  tolua_tousertype(tolua_S,1,0);
#ifndef TOLUA_RELEASE
  if (!self) tolua_error(tolua_S,"invalid 'self' in function 'msgq_try_lock'",NULL);
#endif
  {
   bool tolua_ret = (bool)  self->msgq_try_lock();
   tolua_pushboolean(tolua_S,(bool)tolua_ret);
  }
 }
 return 1;
#ifndef TOLUA_RELEASE
 tolua_lerror:
 tolua_error(tolua_S,"#ferror in function 'msgq_try_lock'.",&tolua_err);
 return 0;
#endif
}
#endif //#ifndef TOLUA_DISABLE

/* method: msgq_unlock of class  Interface */
#ifndef TOLUA_DISABLE_tolua_interface_Interface_msgq_unlock00
static int tolua_interface_Interface_msgq_unlock00(lua_State* tolua_S)
{
#ifndef TOLUA_RELEASE
 tolua_Error tolua_err;
 if (
     !tolua_isusertype(tolua_S,1,"Interface",0,&tolua_err) ||
     !tolua_isnoobj(tolua_S,2,&tolua_err)
 )
  goto tolua_lerror;
 else
#endif
 {
  Interface* self = (Interface*)  tolua_tousertype(tolua_S,1,0);
#ifndef TOLUA_RELEASE
  if (!self) tolua_error(tolua_S,"invalid 'self' in function 'msgq_unlock'",NULL);
#endif
  {
   self->msgq_unlock();
  }
 }
 return 0;
#ifndef TOLUA_RELEASE
 tolua_lerror:
 tolua_error(tolua_S,"#ferror in function 'msgq_unlock'.",&tolua_err);
 return 0;
#endif
}
#endif //#ifndef TOLUA_DISABLE

/* method: msgq_pop of class  Interface */
#ifndef TOLUA_DISABLE_tolua_interface_Interface_msgq_pop00
static int tolua_interface_Interface_msgq_pop00(lua_State* tolua_S)
{
#ifndef TOLUA_RELEASE
 tolua_Error tolua_err;
 if (
     !tolua_isusertype(tolua_S,1,"Interface",0,&tolua_err) ||
     !tolua_isnoobj(tolua_S,2,&tolua_err)
 )
  goto tolua_lerror;
 else
#endif
 {
  Interface* self = (Interface*)  tolua_tousertype(tolua_S,1,0);
#ifndef TOLUA_RELEASE
  if (!self) tolua_error(tolua_S,"invalid 'self' in function 'msgq_pop'",NULL);
#endif
  {
   self->msgq_pop();
  }
 }
 return 0;
#ifndef TOLUA_RELEASE
 tolua_lerror:
 tolua_error(tolua_S,"#ferror in function 'msgq_pop'.",&tolua_err);
 return 0;
#endif
}
#endif //#ifndef TOLUA_DISABLE

/* method: msgq_first of class  Interface */
#ifndef TOLUA_DISABLE_tolua_interface_Interface_msgq_first00
static int tolua_interface_Interface_msgq_first00(lua_State* tolua_S)
{
#ifndef TOLUA_RELEASE
 tolua_Error tolua_err;
 if (
     !tolua_isusertype(tolua_S,1,"Interface",0,&tolua_err) ||
     !tolua_isnoobj(tolua_S,2,&tolua_err)
 )
  goto tolua_lerror;
 else
#endif
 {
  Interface* self = (Interface*)  tolua_tousertype(tolua_S,1,0);
#ifndef TOLUA_RELEASE
  if (!self) tolua_error(tolua_S,"invalid 'self' in function 'msgq_first'",NULL);
#endif
  {
   Message* tolua_ret = (Message*)  self->msgq_first();
   tolua_pushusertype(tolua_S,(void*)tolua_ret,"Message");
  }
 }
 return 1;
#ifndef TOLUA_RELEASE
 tolua_lerror:
 tolua_error(tolua_S,"#ferror in function 'msgq_first'.",&tolua_err);
 return 0;
#endif
}
#endif //#ifndef TOLUA_DISABLE

/* method: msgq_empty of class  Interface */
#ifndef TOLUA_DISABLE_tolua_interface_Interface_msgq_empty00
static int tolua_interface_Interface_msgq_empty00(lua_State* tolua_S)
{
#ifndef TOLUA_RELEASE
 tolua_Error tolua_err;
 if (
     !tolua_isusertype(tolua_S,1,"Interface",0,&tolua_err) ||
     !tolua_isnoobj(tolua_S,2,&tolua_err)
 )
  goto tolua_lerror;
 else
#endif
 {
  Interface* self = (Interface*)  tolua_tousertype(tolua_S,1,0);
#ifndef TOLUA_RELEASE
  if (!self) tolua_error(tolua_S,"invalid 'self' in function 'msgq_empty'",NULL);
#endif
  {
   bool tolua_ret = (bool)  self->msgq_empty();
   tolua_pushboolean(tolua_S,(bool)tolua_ret);
  }
 }
 return 1;
#ifndef TOLUA_RELEASE
 tolua_lerror:
 tolua_error(tolua_S,"#ferror in function 'msgq_empty'.",&tolua_err);
 return 0;
#endif
}
#endif //#ifndef TOLUA_DISABLE

/* method: new of class  MessageQueue */
#ifndef TOLUA_DISABLE_tolua_interface_MessageQueue_new00
static int tolua_interface_MessageQueue_new00(lua_State* tolua_S)
{
#ifndef TOLUA_RELEASE
 tolua_Error tolua_err;
 if (
     !tolua_isusertable(tolua_S,1,"MessageQueue",0,&tolua_err) ||
     !tolua_isnoobj(tolua_S,2,&tolua_err)
 )
  goto tolua_lerror;
 else
#endif
 {
  {
   MessageQueue* tolua_ret = (MessageQueue*)  new MessageQueue();
   tolua_pushusertype(tolua_S,(void*)tolua_ret,"MessageQueue");
  }
 }
 return 1;
#ifndef TOLUA_RELEASE
 tolua_lerror:
 tolua_error(tolua_S,"#ferror in function 'new'.",&tolua_err);
 return 0;
#endif
}
#endif //#ifndef TOLUA_DISABLE

/* method: new_local of class  MessageQueue */
#ifndef TOLUA_DISABLE_tolua_interface_MessageQueue_new00_local
static int tolua_interface_MessageQueue_new00_local(lua_State* tolua_S)
{
#ifndef TOLUA_RELEASE
 tolua_Error tolua_err;
 if (
     !tolua_isusertable(tolua_S,1,"MessageQueue",0,&tolua_err) ||
     !tolua_isnoobj(tolua_S,2,&tolua_err)
 )
  goto tolua_lerror;
 else
#endif
 {
  {
   MessageQueue* tolua_ret = (MessageQueue*)  new MessageQueue();
   tolua_pushusertype_and_takeownership(tolua_S,(void *)tolua_ret,"MessageQueue");
  }
 }
 return 1;
#ifndef TOLUA_RELEASE
 tolua_lerror:
 tolua_error(tolua_S,"#ferror in function 'new'.",&tolua_err);
 return 0;
#endif
}
#endif //#ifndef TOLUA_DISABLE

/* method: delete of class  MessageQueue */
#ifndef TOLUA_DISABLE_tolua_interface_MessageQueue_delete00
static int tolua_interface_MessageQueue_delete00(lua_State* tolua_S)
{
#ifndef TOLUA_RELEASE
 tolua_Error tolua_err;
 if (
     !tolua_isusertype(tolua_S,1,"MessageQueue",0,&tolua_err) ||
     !tolua_isnoobj(tolua_S,2,&tolua_err)
 )
  goto tolua_lerror;
 else
#endif
 {
  MessageQueue* self = (MessageQueue*)  tolua_tousertype(tolua_S,1,0);
#ifndef TOLUA_RELEASE
  if (!self) tolua_error(tolua_S,"invalid 'self' in function 'delete'",NULL);
#endif
  delete self;
 }
 return 0;
#ifndef TOLUA_RELEASE
 tolua_lerror:
 tolua_error(tolua_S,"#ferror in function 'delete'.",&tolua_err);
 return 0;
#endif
}
#endif //#ifndef TOLUA_DISABLE

/* method: new of class  MessageIterator */
#ifndef TOLUA_DISABLE_tolua_interface_MessageQueue_MessageIterator_new00
static int tolua_interface_MessageQueue_MessageIterator_new00(lua_State* tolua_S)
{
#ifndef TOLUA_RELEASE
 tolua_Error tolua_err;
 if (
     !tolua_isusertable(tolua_S,1,"MessageQueue::MessageIterator",0,&tolua_err) ||
     !tolua_isnoobj(tolua_S,2,&tolua_err)
 )
  goto tolua_lerror;
 else
#endif
 {
  {
   MessageQueue::MessageIterator* tolua_ret = (MessageQueue::MessageIterator*)  new MessageQueue::MessageIterator();
   tolua_pushusertype(tolua_S,(void*)tolua_ret,"MessageQueue::MessageIterator");
  }
 }
 return 1;
#ifndef TOLUA_RELEASE
 tolua_lerror:
 tolua_error(tolua_S,"#ferror in function 'new'.",&tolua_err);
 return 0;
#endif
}
#endif //#ifndef TOLUA_DISABLE

/* method: new_local of class  MessageIterator */
#ifndef TOLUA_DISABLE_tolua_interface_MessageQueue_MessageIterator_new00_local
static int tolua_interface_MessageQueue_MessageIterator_new00_local(lua_State* tolua_S)
{
#ifndef TOLUA_RELEASE
 tolua_Error tolua_err;
 if (
     !tolua_isusertable(tolua_S,1,"MessageQueue::MessageIterator",0,&tolua_err) ||
     !tolua_isnoobj(tolua_S,2,&tolua_err)
 )
  goto tolua_lerror;
 else
#endif
 {
  {
   MessageQueue::MessageIterator* tolua_ret = (MessageQueue::MessageIterator*)  new MessageQueue::MessageIterator();
   tolua_pushusertype_and_takeownership(tolua_S,(void *)tolua_ret,"MessageQueue::MessageIterator");
  }
 }
 return 1;
#ifndef TOLUA_RELEASE
 tolua_lerror:
 tolua_error(tolua_S,"#ferror in function 'new'.",&tolua_err);
 return 0;
#endif
}
#endif //#ifndef TOLUA_DISABLE

/* method: new of class  MessageIterator */
#ifndef TOLUA_DISABLE_tolua_interface_MessageQueue_MessageIterator_new01
static int tolua_interface_MessageQueue_MessageIterator_new01(lua_State* tolua_S)
{
 tolua_Error tolua_err;
 if (
     !tolua_isusertable(tolua_S,1,"MessageQueue::MessageIterator",0,&tolua_err) ||
     !tolua_isusertype(tolua_S,2,"const MessageQueue::MessageIterator",0,&tolua_err) ||
     !tolua_isnoobj(tolua_S,3,&tolua_err)
 )
  goto tolua_lerror;
 else
 {
  const MessageQueue::MessageIterator* it = ((const MessageQueue::MessageIterator*)  tolua_tousertype(tolua_S,2,0));
  {
   MessageQueue::MessageIterator* tolua_ret = (MessageQueue::MessageIterator*)  new MessageQueue::MessageIterator(*it);
   tolua_pushusertype(tolua_S,(void*)tolua_ret,"MessageQueue::MessageIterator");
  }
 }
 return 1;
tolua_lerror:
 return tolua_interface_MessageQueue_MessageIterator_new00(tolua_S);
}
#endif //#ifndef TOLUA_DISABLE

/* method: new_local of class  MessageIterator */
#ifndef TOLUA_DISABLE_tolua_interface_MessageQueue_MessageIterator_new01_local
static int tolua_interface_MessageQueue_MessageIterator_new01_local(lua_State* tolua_S)
{
 tolua_Error tolua_err;
 if (
     !tolua_isusertable(tolua_S,1,"MessageQueue::MessageIterator",0,&tolua_err) ||
     !tolua_isusertype(tolua_S,2,"const MessageQueue::MessageIterator",0,&tolua_err) ||
     !tolua_isnoobj(tolua_S,3,&tolua_err)
 )
  goto tolua_lerror;
 else
 {
  const MessageQueue::MessageIterator* it = ((const MessageQueue::MessageIterator*)  tolua_tousertype(tolua_S,2,0));
  {
   MessageQueue::MessageIterator* tolua_ret = (MessageQueue::MessageIterator*)  new MessageQueue::MessageIterator(*it);
   tolua_pushusertype_and_takeownership(tolua_S,(void *)tolua_ret,"MessageQueue::MessageIterator");
  }
 }
 return 1;
tolua_lerror:
 return tolua_interface_MessageQueue_MessageIterator_new00_local(tolua_S);
}
#endif //#ifndef TOLUA_DISABLE

/* method: operator+ of class  MessageIterator */
#ifndef TOLUA_DISABLE_tolua_interface_MessageQueue_MessageIterator__add00
static int tolua_interface_MessageQueue_MessageIterator__add00(lua_State* tolua_S)
{
#ifndef TOLUA_RELEASE
 tolua_Error tolua_err;
 if (
     !tolua_isusertype(tolua_S,1,"MessageQueue::MessageIterator",0,&tolua_err) ||
     !tolua_isnumber(tolua_S,2,0,&tolua_err) ||
     !tolua_isnoobj(tolua_S,3,&tolua_err)
 )
  goto tolua_lerror;
 else
#endif
 {
  MessageQueue::MessageIterator* self = (MessageQueue::MessageIterator*)  tolua_tousertype(tolua_S,1,0);
  unsigned int i = ((unsigned int)  tolua_tonumber(tolua_S,2,0));
#ifndef TOLUA_RELEASE
  if (!self) tolua_error(tolua_S,"invalid 'self' in function 'operator+'",NULL);
#endif
  {
   MessageQueue::MessageIterator& tolua_ret = (MessageQueue::MessageIterator&)  self->operator+(i);
   tolua_pushusertype(tolua_S,(void*)&tolua_ret,"MessageQueue::MessageIterator");
  }
 }
 return 1;
#ifndef TOLUA_RELEASE
 tolua_lerror:
 tolua_error(tolua_S,"#ferror in function '.add'.",&tolua_err);
 return 0;
#endif
}
#endif //#ifndef TOLUA_DISABLE

/* method: operator== of class  MessageIterator */
#ifndef TOLUA_DISABLE_tolua_interface_MessageQueue_MessageIterator__eq00
static int tolua_interface_MessageQueue_MessageIterator__eq00(lua_State* tolua_S)
{
#ifndef TOLUA_RELEASE
 tolua_Error tolua_err;
 if (
     !tolua_isusertype(tolua_S,1,"const MessageQueue::MessageIterator",0,&tolua_err) ||
     !tolua_isusertype(tolua_S,2,"const MessageQueue::MessageIterator",0,&tolua_err) ||
     !tolua_isnoobj(tolua_S,3,&tolua_err)
 )
  goto tolua_lerror;
 else
#endif
 {
  const MessageQueue::MessageIterator* self = (const MessageQueue::MessageIterator*)  tolua_tousertype(tolua_S,1,0);
  const MessageQueue::MessageIterator* c = ((const MessageQueue::MessageIterator*)  tolua_tousertype(tolua_S,2,0));
#ifndef TOLUA_RELEASE
  if (!self) tolua_error(tolua_S,"invalid 'self' in function 'operator=='",NULL);
#endif
  {
   bool tolua_ret = (bool)  self->operator==(*c);
   tolua_pushboolean(tolua_S,(bool)tolua_ret);
  }
 }
 return 1;
#ifndef TOLUA_RELEASE
 tolua_lerror:
 tolua_error(tolua_S,"#ferror in function '.eq'.",&tolua_err);
 return 0;
#endif
}
#endif //#ifndef TOLUA_DISABLE

/* method: operator* of class  MessageIterator */
#ifndef TOLUA_DISABLE_tolua_interface_MessageQueue_MessageIterator__mul00
static int tolua_interface_MessageQueue_MessageIterator__mul00(lua_State* tolua_S)
{
#ifndef TOLUA_RELEASE
 tolua_Error tolua_err;
 if (
     !tolua_isusertype(tolua_S,1,"const MessageQueue::MessageIterator",0,&tolua_err) ||
     !tolua_isnoobj(tolua_S,2,&tolua_err)
 )
  goto tolua_lerror;
 else
#endif
 {
  const MessageQueue::MessageIterator* self = (const MessageQueue::MessageIterator*)  tolua_tousertype(tolua_S,1,0);
#ifndef TOLUA_RELEASE
  if (!self) tolua_error(tolua_S,"invalid 'self' in function 'operator*'",NULL);
#endif
  {
   Message* tolua_ret = (Message*)  self->operator*();
   tolua_pushusertype(tolua_S,(void*)tolua_ret,"Message");
  }
 }
 return 1;
#ifndef TOLUA_RELEASE
 tolua_lerror:
 tolua_error(tolua_S,"#ferror in function '.mul'.",&tolua_err);
 return 0;
#endif
}
#endif //#ifndef TOLUA_DISABLE

/* method: id of class  MessageIterator */
#ifndef TOLUA_DISABLE_tolua_interface_MessageQueue_MessageIterator_id00
static int tolua_interface_MessageQueue_MessageIterator_id00(lua_State* tolua_S)
{
#ifndef TOLUA_RELEASE
 tolua_Error tolua_err;
 if (
     !tolua_isusertype(tolua_S,1,"const MessageQueue::MessageIterator",0,&tolua_err) ||
     !tolua_isnoobj(tolua_S,2,&tolua_err)
 )
  goto tolua_lerror;
 else
#endif
 {
  const MessageQueue::MessageIterator* self = (const MessageQueue::MessageIterator*)  tolua_tousertype(tolua_S,1,0);
#ifndef TOLUA_RELEASE
  if (!self) tolua_error(tolua_S,"invalid 'self' in function 'id'",NULL);
#endif
  {
   unsigned int tolua_ret = (unsigned int)  self->id();
   tolua_pushnumber(tolua_S,(lua_Number)tolua_ret);
  }
 }
 return 1;
#ifndef TOLUA_RELEASE
 tolua_lerror:
 tolua_error(tolua_S,"#ferror in function 'id'.",&tolua_err);
 return 0;
#endif
}
#endif //#ifndef TOLUA_DISABLE

/* method: append of class  MessageQueue */
#ifndef TOLUA_DISABLE_tolua_interface_MessageQueue_append00
static int tolua_interface_MessageQueue_append00(lua_State* tolua_S)
{
#ifndef TOLUA_RELEASE
 tolua_Error tolua_err;
 if (
     !tolua_isusertype(tolua_S,1,"MessageQueue",0,&tolua_err) ||
     !tolua_isusertype(tolua_S,2,"Message",0,&tolua_err) ||
     !tolua_isnoobj(tolua_S,3,&tolua_err)
 )
  goto tolua_lerror;
 else
#endif
 {
  MessageQueue* self = (MessageQueue*)  tolua_tousertype(tolua_S,1,0);
  Message* msg = ((Message*)  tolua_tousertype(tolua_S,2,0));
#ifndef TOLUA_RELEASE
  if (!self) tolua_error(tolua_S,"invalid 'self' in function 'append'",NULL);
#endif
  {
   unsigned int tolua_ret = (unsigned int)  self->append(msg);
   tolua_pushnumber(tolua_S,(lua_Number)tolua_ret);
  }
 }
 return 1;
#ifndef TOLUA_RELEASE
 tolua_lerror:
 tolua_error(tolua_S,"#ferror in function 'append'.",&tolua_err);
 return 0;
#endif
}
#endif //#ifndef TOLUA_DISABLE

/* method: remove of class  MessageQueue */
#ifndef TOLUA_DISABLE_tolua_interface_MessageQueue_remove00
static int tolua_interface_MessageQueue_remove00(lua_State* tolua_S)
{
#ifndef TOLUA_RELEASE
 tolua_Error tolua_err;
 if (
     !tolua_isusertype(tolua_S,1,"MessageQueue",0,&tolua_err) ||
     !tolua_isusertype(tolua_S,2,"const Message",0,&tolua_err) ||
     !tolua_isnoobj(tolua_S,3,&tolua_err)
 )
  goto tolua_lerror;
 else
#endif
 {
  MessageQueue* self = (MessageQueue*)  tolua_tousertype(tolua_S,1,0);
  const Message* msg = ((const Message*)  tolua_tousertype(tolua_S,2,0));
#ifndef TOLUA_RELEASE
  if (!self) tolua_error(tolua_S,"invalid 'self' in function 'remove'",NULL);
#endif
  {
   self->remove(msg);
  }
 }
 return 0;
#ifndef TOLUA_RELEASE
 tolua_lerror:
 tolua_error(tolua_S,"#ferror in function 'remove'.",&tolua_err);
 return 0;
#endif
}
#endif //#ifndef TOLUA_DISABLE

/* method: remove of class  MessageQueue */
#ifndef TOLUA_DISABLE_tolua_interface_MessageQueue_remove01
static int tolua_interface_MessageQueue_remove01(lua_State* tolua_S)
{
 tolua_Error tolua_err;
 if (
     !tolua_isusertype(tolua_S,1,"MessageQueue",0,&tolua_err) ||
     !tolua_isnumber(tolua_S,2,0,&tolua_err) ||
     !tolua_isnoobj(tolua_S,3,&tolua_err)
 )
  goto tolua_lerror;
 else
 {
  MessageQueue* self = (MessageQueue*)  tolua_tousertype(tolua_S,1,0);
  unsigned const int msg_id = ((unsigned const int)  tolua_tonumber(tolua_S,2,0));
#ifndef TOLUA_RELEASE
  if (!self) tolua_error(tolua_S,"invalid 'self' in function 'remove'",NULL);
#endif
  {
   self->remove(msg_id);
  }
 }
 return 0;
tolua_lerror:
 return tolua_interface_MessageQueue_remove00(tolua_S);
}
#endif //#ifndef TOLUA_DISABLE

/* method: insert_after of class  MessageQueue */
#ifndef TOLUA_DISABLE_tolua_interface_MessageQueue_insert_after00
static int tolua_interface_MessageQueue_insert_after00(lua_State* tolua_S)
{
#ifndef TOLUA_RELEASE
 tolua_Error tolua_err;
 if (
     !tolua_isusertype(tolua_S,1,"MessageQueue",0,&tolua_err) ||
     !tolua_isusertype(tolua_S,2,"const MessageQueue::MessageIterator",0,&tolua_err) ||
     !tolua_isusertype(tolua_S,3,"Message",0,&tolua_err) ||
     !tolua_isnoobj(tolua_S,4,&tolua_err)
 )
  goto tolua_lerror;
 else
#endif
 {
  MessageQueue* self = (MessageQueue*)  tolua_tousertype(tolua_S,1,0);
  const MessageQueue::MessageIterator* it = ((const MessageQueue::MessageIterator*)  tolua_tousertype(tolua_S,2,0));
  Message* msg = ((Message*)  tolua_tousertype(tolua_S,3,0));
#ifndef TOLUA_RELEASE
  if (!self) tolua_error(tolua_S,"invalid 'self' in function 'insert_after'",NULL);
#endif
  {
   unsigned int tolua_ret = (unsigned int)  self->insert_after(*it,msg);
   tolua_pushnumber(tolua_S,(lua_Number)tolua_ret);
  }
 }
 return 1;
#ifndef TOLUA_RELEASE
 tolua_lerror:
 tolua_error(tolua_S,"#ferror in function 'insert_after'.",&tolua_err);
 return 0;
#endif
}
#endif //#ifndef TOLUA_DISABLE

/* method: size of class  MessageQueue */
#ifndef TOLUA_DISABLE_tolua_interface_MessageQueue_size00
static int tolua_interface_MessageQueue_size00(lua_State* tolua_S)
{
#ifndef TOLUA_RELEASE
 tolua_Error tolua_err;
 if (
     !tolua_isusertype(tolua_S,1,"const MessageQueue",0,&tolua_err) ||
     !tolua_isnoobj(tolua_S,2,&tolua_err)
 )
  goto tolua_lerror;
 else
#endif
 {
  const MessageQueue* self = (const MessageQueue*)  tolua_tousertype(tolua_S,1,0);
#ifndef TOLUA_RELEASE
  if (!self) tolua_error(tolua_S,"invalid 'self' in function 'size'",NULL);
#endif
  {
   unsigned int tolua_ret = (unsigned int)  self->size();
   tolua_pushnumber(tolua_S,(lua_Number)tolua_ret);
  }
 }
 return 1;
#ifndef TOLUA_RELEASE
 tolua_lerror:
 tolua_error(tolua_S,"#ferror in function 'size'.",&tolua_err);
 return 0;
#endif
}
#endif //#ifndef TOLUA_DISABLE

/* method: flush of class  MessageQueue */
#ifndef TOLUA_DISABLE_tolua_interface_MessageQueue_flush00
static int tolua_interface_MessageQueue_flush00(lua_State* tolua_S)
{
#ifndef TOLUA_RELEASE
 tolua_Error tolua_err;
 if (
     !tolua_isusertype(tolua_S,1,"MessageQueue",0,&tolua_err) ||
     !tolua_isnoobj(tolua_S,2,&tolua_err)
 )
  goto tolua_lerror;
 else
#endif
 {
  MessageQueue* self = (MessageQueue*)  tolua_tousertype(tolua_S,1,0);
#ifndef TOLUA_RELEASE
  if (!self) tolua_error(tolua_S,"invalid 'self' in function 'flush'",NULL);
#endif
  {
   self->flush();
  }
 }
 return 0;
#ifndef TOLUA_RELEASE
 tolua_lerror:
 tolua_error(tolua_S,"#ferror in function 'flush'.",&tolua_err);
 return 0;
#endif
}
#endif //#ifndef TOLUA_DISABLE

/* method: empty of class  MessageQueue */
#ifndef TOLUA_DISABLE_tolua_interface_MessageQueue_empty00
static int tolua_interface_MessageQueue_empty00(lua_State* tolua_S)
{
#ifndef TOLUA_RELEASE
 tolua_Error tolua_err;
 if (
     !tolua_isusertype(tolua_S,1,"const MessageQueue",0,&tolua_err) ||
     !tolua_isnoobj(tolua_S,2,&tolua_err)
 )
  goto tolua_lerror;
 else
#endif
 {
  const MessageQueue* self = (const MessageQueue*)  tolua_tousertype(tolua_S,1,0);
#ifndef TOLUA_RELEASE
  if (!self) tolua_error(tolua_S,"invalid 'self' in function 'empty'",NULL);
#endif
  {
   bool tolua_ret = (bool)  self->empty();
   tolua_pushboolean(tolua_S,(bool)tolua_ret);
  }
 }
 return 1;
#ifndef TOLUA_RELEASE
 tolua_lerror:
 tolua_error(tolua_S,"#ferror in function 'empty'.",&tolua_err);
 return 0;
#endif
}
#endif //#ifndef TOLUA_DISABLE

/* method: lock of class  MessageQueue */
#ifndef TOLUA_DISABLE_tolua_interface_MessageQueue_lock00
static int tolua_interface_MessageQueue_lock00(lua_State* tolua_S)
{
#ifndef TOLUA_RELEASE
 tolua_Error tolua_err;
 if (
     !tolua_isusertype(tolua_S,1,"MessageQueue",0,&tolua_err) ||
     !tolua_isnoobj(tolua_S,2,&tolua_err)
 )
  goto tolua_lerror;
 else
#endif
 {
  MessageQueue* self = (MessageQueue*)  tolua_tousertype(tolua_S,1,0);
#ifndef TOLUA_RELEASE
  if (!self) tolua_error(tolua_S,"invalid 'self' in function 'lock'",NULL);
#endif
  {
   self->lock();
  }
 }
 return 0;
#ifndef TOLUA_RELEASE
 tolua_lerror:
 tolua_error(tolua_S,"#ferror in function 'lock'.",&tolua_err);
 return 0;
#endif
}
#endif //#ifndef TOLUA_DISABLE

/* method: try_lock of class  MessageQueue */
#ifndef TOLUA_DISABLE_tolua_interface_MessageQueue_try_lock00
static int tolua_interface_MessageQueue_try_lock00(lua_State* tolua_S)
{
#ifndef TOLUA_RELEASE
 tolua_Error tolua_err;
 if (
     !tolua_isusertype(tolua_S,1,"MessageQueue",0,&tolua_err) ||
     !tolua_isnoobj(tolua_S,2,&tolua_err)
 )
  goto tolua_lerror;
 else
#endif
 {
  MessageQueue* self = (MessageQueue*)  tolua_tousertype(tolua_S,1,0);
#ifndef TOLUA_RELEASE
  if (!self) tolua_error(tolua_S,"invalid 'self' in function 'try_lock'",NULL);
#endif
  {
   bool tolua_ret = (bool)  self->try_lock();
   tolua_pushboolean(tolua_S,(bool)tolua_ret);
  }
 }
 return 1;
#ifndef TOLUA_RELEASE
 tolua_lerror:
 tolua_error(tolua_S,"#ferror in function 'try_lock'.",&tolua_err);
 return 0;
#endif
}
#endif //#ifndef TOLUA_DISABLE

/* method: unlock of class  MessageQueue */
#ifndef TOLUA_DISABLE_tolua_interface_MessageQueue_unlock00
static int tolua_interface_MessageQueue_unlock00(lua_State* tolua_S)
{
#ifndef TOLUA_RELEASE
 tolua_Error tolua_err;
 if (
     !tolua_isusertype(tolua_S,1,"MessageQueue",0,&tolua_err) ||
     !tolua_isnoobj(tolua_S,2,&tolua_err)
 )
  goto tolua_lerror;
 else
#endif
 {
  MessageQueue* self = (MessageQueue*)  tolua_tousertype(tolua_S,1,0);
#ifndef TOLUA_RELEASE
  if (!self) tolua_error(tolua_S,"invalid 'self' in function 'unlock'",NULL);
#endif
  {
   self->unlock();
  }
 }
 return 0;
#ifndef TOLUA_RELEASE
 tolua_lerror:
 tolua_error(tolua_S,"#ferror in function 'unlock'.",&tolua_err);
 return 0;
#endif
}
#endif //#ifndef TOLUA_DISABLE

/* method: first of class  MessageQueue */
#ifndef TOLUA_DISABLE_tolua_interface_MessageQueue_first00
static int tolua_interface_MessageQueue_first00(lua_State* tolua_S)
{
#ifndef TOLUA_RELEASE
 tolua_Error tolua_err;
 if (
     !tolua_isusertype(tolua_S,1,"MessageQueue",0,&tolua_err) ||
     !tolua_isnoobj(tolua_S,2,&tolua_err)
 )
  goto tolua_lerror;
 else
#endif
 {
  MessageQueue* self = (MessageQueue*)  tolua_tousertype(tolua_S,1,0);
#ifndef TOLUA_RELEASE
  if (!self) tolua_error(tolua_S,"invalid 'self' in function 'first'",NULL);
#endif
  {
   Message* tolua_ret = (Message*)  self->first();
   tolua_pushusertype(tolua_S,(void*)tolua_ret,"Message");
  }
 }
 return 1;
#ifndef TOLUA_RELEASE
 tolua_lerror:
 tolua_error(tolua_S,"#ferror in function 'first'.",&tolua_err);
 return 0;
#endif
}
#endif //#ifndef TOLUA_DISABLE

/* method: pop of class  MessageQueue */
#ifndef TOLUA_DISABLE_tolua_interface_MessageQueue_pop00
static int tolua_interface_MessageQueue_pop00(lua_State* tolua_S)
{
#ifndef TOLUA_RELEASE
 tolua_Error tolua_err;
 if (
     !tolua_isusertype(tolua_S,1,"MessageQueue",0,&tolua_err) ||
     !tolua_isnoobj(tolua_S,2,&tolua_err)
 )
  goto tolua_lerror;
 else
#endif
 {
  MessageQueue* self = (MessageQueue*)  tolua_tousertype(tolua_S,1,0);
#ifndef TOLUA_RELEASE
  if (!self) tolua_error(tolua_S,"invalid 'self' in function 'pop'",NULL);
#endif
  {
   self->pop();
  }
 }
 return 0;
#ifndef TOLUA_RELEASE
 tolua_lerror:
 tolua_error(tolua_S,"#ferror in function 'pop'.",&tolua_err);
 return 0;
#endif
}
#endif //#ifndef TOLUA_DISABLE

/* method: begin of class  MessageQueue */
#ifndef TOLUA_DISABLE_tolua_interface_MessageQueue_begin00
static int tolua_interface_MessageQueue_begin00(lua_State* tolua_S)
{
#ifndef TOLUA_RELEASE
 tolua_Error tolua_err;
 if (
     !tolua_isusertype(tolua_S,1,"MessageQueue",0,&tolua_err) ||
     !tolua_isnoobj(tolua_S,2,&tolua_err)
 )
  goto tolua_lerror;
 else
#endif
 {
  MessageQueue* self = (MessageQueue*)  tolua_tousertype(tolua_S,1,0);
#ifndef TOLUA_RELEASE
  if (!self) tolua_error(tolua_S,"invalid 'self' in function 'begin'",NULL);
#endif
  {
   MessageQueue::MessageIterator tolua_ret = (MessageQueue::MessageIterator)  self->begin();
   {
#ifdef __cplusplus
    void* tolua_obj = new MessageQueue::MessageIterator(tolua_ret);
    tolua_pushusertype_and_takeownership(tolua_S,tolua_obj,"MessageQueue::MessageIterator");
#else
    void* tolua_obj = tolua_copy(tolua_S,(void*)&tolua_ret,sizeof(MessageQueue::MessageIterator));
    tolua_pushusertype_and_takeownership(tolua_S,tolua_obj,"MessageQueue::MessageIterator");
#endif
   }
  }
 }
 return 1;
#ifndef TOLUA_RELEASE
 tolua_lerror:
 tolua_error(tolua_S,"#ferror in function 'begin'.",&tolua_err);
 return 0;
#endif
}
#endif //#ifndef TOLUA_DISABLE

/* method: end of class  MessageQueue */
#ifndef TOLUA_DISABLE_tolua_interface_MessageQueue_end00
static int tolua_interface_MessageQueue_end00(lua_State* tolua_S)
{
#ifndef TOLUA_RELEASE
 tolua_Error tolua_err;
 if (
     !tolua_isusertype(tolua_S,1,"MessageQueue",0,&tolua_err) ||
     !tolua_isnoobj(tolua_S,2,&tolua_err)
 )
  goto tolua_lerror;
 else
#endif
 {
  MessageQueue* self = (MessageQueue*)  tolua_tousertype(tolua_S,1,0);
#ifndef TOLUA_RELEASE
  if (!self) tolua_error(tolua_S,"invalid 'self' in function 'end'",NULL);
#endif
  {
   MessageQueue::MessageIterator tolua_ret = (MessageQueue::MessageIterator)  self->end();
   {
#ifdef __cplusplus
    void* tolua_obj = new MessageQueue::MessageIterator(tolua_ret);
    tolua_pushusertype_and_takeownership(tolua_S,tolua_obj,"MessageQueue::MessageIterator");
#else
    void* tolua_obj = tolua_copy(tolua_S,(void*)&tolua_ret,sizeof(MessageQueue::MessageIterator));
    tolua_pushusertype_and_takeownership(tolua_S,tolua_obj,"MessageQueue::MessageIterator");
#endif
   }
  }
 }
 return 1;
#ifndef TOLUA_RELEASE
 tolua_lerror:
 tolua_error(tolua_S,"#ferror in function 'end'.",&tolua_err);
 return 0;
#endif
}
#endif //#ifndef TOLUA_DISABLE

/* method: new of class  Message */
#ifndef TOLUA_DISABLE_tolua_interface_Message_new00
static int tolua_interface_Message_new00(lua_State* tolua_S)
{
#ifndef TOLUA_RELEASE
 tolua_Error tolua_err;
 if (
     !tolua_isusertable(tolua_S,1,"Message",0,&tolua_err) ||
     !tolua_isstring(tolua_S,2,0,&tolua_err) ||
     !tolua_isnoobj(tolua_S,3,&tolua_err)
 )
  goto tolua_lerror;
 else
#endif
 {
  const char* type = ((const char*)  tolua_tostring(tolua_S,2,0));
  {
   Message* tolua_ret = (Message*)  new Message(type);
   tolua_pushusertype(tolua_S,(void*)tolua_ret,"Message");
  }
 }
 return 1;
#ifndef TOLUA_RELEASE
 tolua_lerror:
 tolua_error(tolua_S,"#ferror in function 'new'.",&tolua_err);
 return 0;
#endif
}
#endif //#ifndef TOLUA_DISABLE

/* method: new_local of class  Message */
#ifndef TOLUA_DISABLE_tolua_interface_Message_new00_local
static int tolua_interface_Message_new00_local(lua_State* tolua_S)
{
#ifndef TOLUA_RELEASE
 tolua_Error tolua_err;
 if (
     !tolua_isusertable(tolua_S,1,"Message",0,&tolua_err) ||
     !tolua_isstring(tolua_S,2,0,&tolua_err) ||
     !tolua_isnoobj(tolua_S,3,&tolua_err)
 )
  goto tolua_lerror;
 else
#endif
 {
  const char* type = ((const char*)  tolua_tostring(tolua_S,2,0));
  {
   Message* tolua_ret = (Message*)  new Message(type);
   tolua_pushusertype_and_takeownership(tolua_S,(void *)tolua_ret,"Message");
  }
 }
 return 1;
#ifndef TOLUA_RELEASE
 tolua_lerror:
 tolua_error(tolua_S,"#ferror in function 'new'.",&tolua_err);
 return 0;
#endif
}
#endif //#ifndef TOLUA_DISABLE

/* method: new of class  Message */
#ifndef TOLUA_DISABLE_tolua_interface_Message_new01
static int tolua_interface_Message_new01(lua_State* tolua_S)
{
 tolua_Error tolua_err;
 if (
     !tolua_isusertable(tolua_S,1,"Message",0,&tolua_err) ||
     !tolua_isusertype(tolua_S,2,"Message",0,&tolua_err) ||
     !tolua_isnoobj(tolua_S,3,&tolua_err)
 )
  goto tolua_lerror;
 else
 {
  Message* mesg = ((Message*)  tolua_tousertype(tolua_S,2,0));
  {
   Message* tolua_ret = (Message*)  new Message(mesg);
   tolua_pushusertype(tolua_S,(void*)tolua_ret,"Message");
  }
 }
 return 1;
tolua_lerror:
 return tolua_interface_Message_new00(tolua_S);
}
#endif //#ifndef TOLUA_DISABLE

/* method: new_local of class  Message */
#ifndef TOLUA_DISABLE_tolua_interface_Message_new01_local
static int tolua_interface_Message_new01_local(lua_State* tolua_S)
{
 tolua_Error tolua_err;
 if (
     !tolua_isusertable(tolua_S,1,"Message",0,&tolua_err) ||
     !tolua_isusertype(tolua_S,2,"Message",0,&tolua_err) ||
     !tolua_isnoobj(tolua_S,3,&tolua_err)
 )
  goto tolua_lerror;
 else
 {
  Message* mesg = ((Message*)  tolua_tousertype(tolua_S,2,0));
  {
   Message* tolua_ret = (Message*)  new Message(mesg);
   tolua_pushusertype_and_takeownership(tolua_S,(void *)tolua_ret,"Message");
  }
 }
 return 1;
tolua_lerror:
 return tolua_interface_Message_new00_local(tolua_S);
}
#endif //#ifndef TOLUA_DISABLE

/* method: new of class  Message */
#ifndef TOLUA_DISABLE_tolua_interface_Message_new02
static int tolua_interface_Message_new02(lua_State* tolua_S)
{
 tolua_Error tolua_err;
 if (
     !tolua_isusertable(tolua_S,1,"Message",0,&tolua_err) ||
     !tolua_isusertype(tolua_S,2,"Message",0,&tolua_err) ||
     !tolua_isnoobj(tolua_S,3,&tolua_err)
 )
  goto tolua_lerror;
 else
 {
  Message* mesg = ((Message*)  tolua_tousertype(tolua_S,2,0));
  {
   Message* tolua_ret = (Message*)  new Message(*mesg);
   tolua_pushusertype(tolua_S,(void*)tolua_ret,"Message");
  }
 }
 return 1;
tolua_lerror:
 return tolua_interface_Message_new01(tolua_S);
}
#endif //#ifndef TOLUA_DISABLE

/* method: new_local of class  Message */
#ifndef TOLUA_DISABLE_tolua_interface_Message_new02_local
static int tolua_interface_Message_new02_local(lua_State* tolua_S)
{
 tolua_Error tolua_err;
 if (
     !tolua_isusertable(tolua_S,1,"Message",0,&tolua_err) ||
     !tolua_isusertype(tolua_S,2,"Message",0,&tolua_err) ||
     !tolua_isnoobj(tolua_S,3,&tolua_err)
 )
  goto tolua_lerror;
 else
 {
  Message* mesg = ((Message*)  tolua_tousertype(tolua_S,2,0));
  {
   Message* tolua_ret = (Message*)  new Message(*mesg);
   tolua_pushusertype_and_takeownership(tolua_S,(void *)tolua_ret,"Message");
  }
 }
 return 1;
tolua_lerror:
 return tolua_interface_Message_new01_local(tolua_S);
}
#endif //#ifndef TOLUA_DISABLE

/* method: set_status of class  Message */
#ifndef TOLUA_DISABLE_tolua_interface_Message_set_status00
static int tolua_interface_Message_set_status00(lua_State* tolua_S)
{
#ifndef TOLUA_RELEASE
 tolua_Error tolua_err;
 if (
     !tolua_isusertype(tolua_S,1,"Message",0,&tolua_err) ||
     !tolua_isnumber(tolua_S,2,0,&tolua_err) ||
     !tolua_isnoobj(tolua_S,3,&tolua_err)
 )
  goto tolua_lerror;
 else
#endif
 {
  Message* self = (Message*)  tolua_tousertype(tolua_S,1,0);
  Message::MessageStatus status = ((Message::MessageStatus) (int)  tolua_tonumber(tolua_S,2,0));
#ifndef TOLUA_RELEASE
  if (!self) tolua_error(tolua_S,"invalid 'self' in function 'set_status'",NULL);
#endif
  {
   self->set_status(status);
  }
 }
 return 0;
#ifndef TOLUA_RELEASE
 tolua_lerror:
 tolua_error(tolua_S,"#ferror in function 'set_status'.",&tolua_err);
 return 0;
#endif
}
#endif //#ifndef TOLUA_DISABLE

/* method: status of class  Message */
#ifndef TOLUA_DISABLE_tolua_interface_Message_status00
static int tolua_interface_Message_status00(lua_State* tolua_S)
{
#ifndef TOLUA_RELEASE
 tolua_Error tolua_err;
 if (
     !tolua_isusertype(tolua_S,1,"const Message",0,&tolua_err) ||
     !tolua_isnoobj(tolua_S,2,&tolua_err)
 )
  goto tolua_lerror;
 else
#endif
 {
  const Message* self = (const Message*)  tolua_tousertype(tolua_S,1,0);
#ifndef TOLUA_RELEASE
  if (!self) tolua_error(tolua_S,"invalid 'self' in function 'status'",NULL);
#endif
  {
   Message::MessageStatus tolua_ret = (Message::MessageStatus)  self->status();
   tolua_pushnumber(tolua_S,(lua_Number)tolua_ret);
  }
 }
 return 1;
#ifndef TOLUA_RELEASE
 tolua_lerror:
 tolua_error(tolua_S,"#ferror in function 'status'.",&tolua_err);
 return 0;
#endif
}
#endif //#ifndef TOLUA_DISABLE

/* method: set_sub_status of class  Message */
#ifndef TOLUA_DISABLE_tolua_interface_Message_set_sub_status00
static int tolua_interface_Message_set_sub_status00(lua_State* tolua_S)
{
#ifndef TOLUA_RELEASE
 tolua_Error tolua_err;
 if (
     !tolua_isusertype(tolua_S,1,"Message",0,&tolua_err) ||
     !tolua_isnumber(tolua_S,2,0,&tolua_err) ||
     !tolua_isnoobj(tolua_S,3,&tolua_err)
 )
  goto tolua_lerror;
 else
#endif
 {
  Message* self = (Message*)  tolua_tousertype(tolua_S,1,0);
  unsigned int sub_status = ((unsigned int)  tolua_tonumber(tolua_S,2,0));
#ifndef TOLUA_RELEASE
  if (!self) tolua_error(tolua_S,"invalid 'self' in function 'set_sub_status'",NULL);
#endif
  {
   self->set_sub_status(sub_status);
  }
 }
 return 0;
#ifndef TOLUA_RELEASE
 tolua_lerror:
 tolua_error(tolua_S,"#ferror in function 'set_sub_status'.",&tolua_err);
 return 0;
#endif
}
#endif //#ifndef TOLUA_DISABLE

/* method: sub_status of class  Message */
#ifndef TOLUA_DISABLE_tolua_interface_Message_sub_status00
static int tolua_interface_Message_sub_status00(lua_State* tolua_S)
{
#ifndef TOLUA_RELEASE
 tolua_Error tolua_err;
 if (
     !tolua_isusertype(tolua_S,1,"const Message",0,&tolua_err) ||
     !tolua_isnoobj(tolua_S,2,&tolua_err)
 )
  goto tolua_lerror;
 else
#endif
 {
  const Message* self = (const Message*)  tolua_tousertype(tolua_S,1,0);
#ifndef TOLUA_RELEASE
  if (!self) tolua_error(tolua_S,"invalid 'self' in function 'sub_status'",NULL);
#endif
  {
   unsigned int tolua_ret = (unsigned int)  self->sub_status();
   tolua_pushnumber(tolua_S,(lua_Number)tolua_ret);
  }
 }
 return 1;
#ifndef TOLUA_RELEASE
 tolua_lerror:
 tolua_error(tolua_S,"#ferror in function 'sub_status'.",&tolua_err);
 return 0;
#endif
}
#endif //#ifndef TOLUA_DISABLE

/* method: sender_id of class  Message */
#ifndef TOLUA_DISABLE_tolua_interface_Message_sender_id00
static int tolua_interface_Message_sender_id00(lua_State* tolua_S)
{
#ifndef TOLUA_RELEASE
 tolua_Error tolua_err;
 if (
     !tolua_isusertype(tolua_S,1,"const Message",0,&tolua_err) ||
     !tolua_isnoobj(tolua_S,2,&tolua_err)
 )
  goto tolua_lerror;
 else
#endif
 {
  const Message* self = (const Message*)  tolua_tousertype(tolua_S,1,0);
#ifndef TOLUA_RELEASE
  if (!self) tolua_error(tolua_S,"invalid 'self' in function 'sender_id'",NULL);
#endif
  {
   unsigned int tolua_ret = (unsigned int)  self->sender_id();
   tolua_pushnumber(tolua_S,(lua_Number)tolua_ret);
  }
 }
 return 1;
#ifndef TOLUA_RELEASE
 tolua_lerror:
 tolua_error(tolua_S,"#ferror in function 'sender_id'.",&tolua_err);
 return 0;
#endif
}
#endif //#ifndef TOLUA_DISABLE

/* method: sender_thread_name of class  Message */
#ifndef TOLUA_DISABLE_tolua_interface_Message_sender_thread_name00
static int tolua_interface_Message_sender_thread_name00(lua_State* tolua_S)
{
#ifndef TOLUA_RELEASE
 tolua_Error tolua_err;
 if (
     !tolua_isusertype(tolua_S,1,"const Message",0,&tolua_err) ||
     !tolua_isnoobj(tolua_S,2,&tolua_err)
 )
  goto tolua_lerror;
 else
#endif
 {
  const Message* self = (const Message*)  tolua_tousertype(tolua_S,1,0);
#ifndef TOLUA_RELEASE
  if (!self) tolua_error(tolua_S,"invalid 'self' in function 'sender_thread_name'",NULL);
#endif
  {
   const char* tolua_ret = (const char*)  self->sender_thread_name();
   tolua_pushstring(tolua_S,(const char*)tolua_ret);
  }
 }
 return 1;
#ifndef TOLUA_RELEASE
 tolua_lerror:
 tolua_error(tolua_S,"#ferror in function 'sender_thread_name'.",&tolua_err);
 return 0;
#endif
}
#endif //#ifndef TOLUA_DISABLE

/* method: interface of class  Message */
#ifndef TOLUA_DISABLE_tolua_interface_Message_interface00
static int tolua_interface_Message_interface00(lua_State* tolua_S)
{
#ifndef TOLUA_RELEASE
 tolua_Error tolua_err;
 if (
     !tolua_isusertype(tolua_S,1,"const Message",0,&tolua_err) ||
     !tolua_isnoobj(tolua_S,2,&tolua_err)
 )
  goto tolua_lerror;
 else
#endif
 {
  const Message* self = (const Message*)  tolua_tousertype(tolua_S,1,0);
#ifndef TOLUA_RELEASE
  if (!self) tolua_error(tolua_S,"invalid 'self' in function 'interface'",NULL);
#endif
  {
   Interface* tolua_ret = (Interface*)  self->interface();
   tolua_pushusertype(tolua_S,(void*)tolua_ret,"Interface");
  }
 }
 return 1;
#ifndef TOLUA_RELEASE
 tolua_lerror:
 tolua_error(tolua_S,"#ferror in function 'interface'.",&tolua_err);
 return 0;
#endif
}
#endif //#ifndef TOLUA_DISABLE

/* method: type of class  Message */
#ifndef TOLUA_DISABLE_tolua_interface_Message_type00
static int tolua_interface_Message_type00(lua_State* tolua_S)
{
#ifndef TOLUA_RELEASE
 tolua_Error tolua_err;
 if (
     !tolua_isusertype(tolua_S,1,"const Message",0,&tolua_err) ||
     !tolua_isnoobj(tolua_S,2,&tolua_err)
 )
  goto tolua_lerror;
 else
#endif
 {
  const Message* self = (const Message*)  tolua_tousertype(tolua_S,1,0);
#ifndef TOLUA_RELEASE
  if (!self) tolua_error(tolua_S,"invalid 'self' in function 'type'",NULL);
#endif
  {
   const char* tolua_ret = (const char*)  self->type();
   tolua_pushstring(tolua_S,(const char*)tolua_ret);
  }
 }
 return 1;
#ifndef TOLUA_RELEASE
 tolua_lerror:
 tolua_error(tolua_S,"#ferror in function 'type'.",&tolua_err);
 return 0;
#endif
}
#endif //#ifndef TOLUA_DISABLE

/* method: datachunk of class  Message */
#ifndef TOLUA_DISABLE_tolua_interface_Message_datachunk00
static int tolua_interface_Message_datachunk00(lua_State* tolua_S)
{
#ifndef TOLUA_RELEASE
 tolua_Error tolua_err;
 if (
     !tolua_isusertype(tolua_S,1,"const Message",0,&tolua_err) ||
     !tolua_isnoobj(tolua_S,2,&tolua_err)
 )
  goto tolua_lerror;
 else
#endif
 {
  const Message* self = (const Message*)  tolua_tousertype(tolua_S,1,0);
#ifndef TOLUA_RELEASE
  if (!self) tolua_error(tolua_S,"invalid 'self' in function 'datachunk'",NULL);
#endif
  {
   const void* tolua_ret = (const void*)  self->datachunk();
   tolua_pushuserdata(tolua_S,(void*)tolua_ret);
  }
 }
 return 1;
#ifndef TOLUA_RELEASE
 tolua_lerror:
 tolua_error(tolua_S,"#ferror in function 'datachunk'.",&tolua_err);
 return 0;
#endif
}
#endif //#ifndef TOLUA_DISABLE

/* method: datasize of class  Message */
#ifndef TOLUA_DISABLE_tolua_interface_Message_datasize00
static int tolua_interface_Message_datasize00(lua_State* tolua_S)
{
#ifndef TOLUA_RELEASE
 tolua_Error tolua_err;
 if (
     !tolua_isusertype(tolua_S,1,"const Message",0,&tolua_err) ||
     !tolua_isnoobj(tolua_S,2,&tolua_err)
 )
  goto tolua_lerror;
 else
#endif
 {
  const Message* self = (const Message*)  tolua_tousertype(tolua_S,1,0);
#ifndef TOLUA_RELEASE
  if (!self) tolua_error(tolua_S,"invalid 'self' in function 'datasize'",NULL);
#endif
  {
   unsigned int tolua_ret = (unsigned int)  self->datasize();
   tolua_pushnumber(tolua_S,(lua_Number)tolua_ret);
  }
 }
 return 1;
#ifndef TOLUA_RELEASE
 tolua_lerror:
 tolua_error(tolua_S,"#ferror in function 'datasize'.",&tolua_err);
 return 0;
#endif
}
#endif //#ifndef TOLUA_DISABLE

/* method: set_from_chunk of class  Message */
#ifndef TOLUA_DISABLE_tolua_interface_Message_set_from_chunk00
static int tolua_interface_Message_set_from_chunk00(lua_State* tolua_S)
{
#ifndef TOLUA_RELEASE
 tolua_Error tolua_err;
 if (
     !tolua_isusertype(tolua_S,1,"Message",0,&tolua_err) ||
     !tolua_isuserdata(tolua_S,2,0,&tolua_err) ||
     !tolua_isnoobj(tolua_S,3,&tolua_err)
 )
  goto tolua_lerror;
 else
#endif
 {
  Message* self = (Message*)  tolua_tousertype(tolua_S,1,0);
  const void* chunk = ((const void*)  tolua_touserdata(tolua_S,2,0));
#ifndef TOLUA_RELEASE
  if (!self) tolua_error(tolua_S,"invalid 'self' in function 'set_from_chunk'",NULL);
#endif
  {
   self->set_from_chunk(chunk);
  }
 }
 return 0;
#ifndef TOLUA_RELEASE
 tolua_lerror:
 tolua_error(tolua_S,"#ferror in function 'set_from_chunk'.",&tolua_err);
 return 0;
#endif
}
#endif //#ifndef TOLUA_DISABLE

/* method: ref of class  Message */
#ifndef TOLUA_DISABLE_tolua_interface_Message_ref00
static int tolua_interface_Message_ref00(lua_State* tolua_S)
{
#ifndef TOLUA_RELEASE
 tolua_Error tolua_err;
 if (
     !tolua_isusertype(tolua_S,1,"Message",0,&tolua_err) ||
     !tolua_isnoobj(tolua_S,2,&tolua_err)
 )
  goto tolua_lerror;
 else
#endif
 {
  Message* self = (Message*)  tolua_tousertype(tolua_S,1,0);
#ifndef TOLUA_RELEASE
  if (!self) tolua_error(tolua_S,"invalid 'self' in function 'ref'",NULL);
#endif
  {
   self->ref();
  }
 }
 return 0;
#ifndef TOLUA_RELEASE
 tolua_lerror:
 tolua_error(tolua_S,"#ferror in function 'ref'.",&tolua_err);
 return 0;
#endif
}
#endif //#ifndef TOLUA_DISABLE

/* method: unref of class  Message */
#ifndef TOLUA_DISABLE_tolua_interface_Message_unref00
static int tolua_interface_Message_unref00(lua_State* tolua_S)
{
#ifndef TOLUA_RELEASE
 tolua_Error tolua_err;
 if (
     !tolua_isusertype(tolua_S,1,"Message",0,&tolua_err) ||
     !tolua_isnoobj(tolua_S,2,&tolua_err)
 )
  goto tolua_lerror;
 else
#endif
 {
  Message* self = (Message*)  tolua_tousertype(tolua_S,1,0);
#ifndef TOLUA_RELEASE
  if (!self) tolua_error(tolua_S,"invalid 'self' in function 'unref'",NULL);
#endif
  {
   self->unref();
  }
 }
 return 0;
#ifndef TOLUA_RELEASE
 tolua_lerror:
 tolua_error(tolua_S,"#ferror in function 'unref'.",&tolua_err);
 return 0;
#endif
}
#endif //#ifndef TOLUA_DISABLE

/* method: refcount of class  Message */
#ifndef TOLUA_DISABLE_tolua_interface_Message_refcount00
static int tolua_interface_Message_refcount00(lua_State* tolua_S)
{
#ifndef TOLUA_RELEASE
 tolua_Error tolua_err;
 if (
     !tolua_isusertype(tolua_S,1,"Message",0,&tolua_err) ||
     !tolua_isnoobj(tolua_S,2,&tolua_err)
 )
  goto tolua_lerror;
 else
#endif
 {
  Message* self = (Message*)  tolua_tousertype(tolua_S,1,0);
#ifndef TOLUA_RELEASE
  if (!self) tolua_error(tolua_S,"invalid 'self' in function 'refcount'",NULL);
#endif
  {
   unsigned int tolua_ret = (unsigned int)  self->refcount();
   tolua_pushnumber(tolua_S,(lua_Number)tolua_ret);
  }
 }
 return 1;
#ifndef TOLUA_RELEASE
 tolua_lerror:
 tolua_error(tolua_S,"#ferror in function 'refcount'.",&tolua_err);
 return 0;
#endif
}
#endif //#ifndef TOLUA_DISABLE

/* Open function */
TOLUA_API int tolua_interface_open (lua_State* tolua_S)
{
 tolua_open(tolua_S);
 tolua_reg_types(tolua_S);
 tolua_module(tolua_S,NULL,0);
 tolua_beginmodule(tolua_S,NULL);
  tolua_cclass(tolua_S,"Interface","Interface","",NULL);
  tolua_beginmodule(tolua_S,"Interface");
   tolua_function(tolua_S,"oftype",tolua_interface_Interface_oftype00);
   tolua_function(tolua_S,"datachunk",tolua_interface_Interface_datachunk00);
   tolua_function(tolua_S,"datasize",tolua_interface_Interface_datasize00);
   tolua_function(tolua_S,"type",tolua_interface_Interface_type00);
   tolua_function(tolua_S,"id",tolua_interface_Interface_id00);
   tolua_function(tolua_S,"uid",tolua_interface_Interface_uid00);
   tolua_function(tolua_S,"serial",tolua_interface_Interface_serial00);
   tolua_function(tolua_S,"mem_serial",tolua_interface_Interface_mem_serial00);
   tolua_function(tolua_S,".eq",tolua_interface_Interface__eq00);
   tolua_function(tolua_S,"hash",tolua_interface_Interface_hash00);
   tolua_function(tolua_S,"hash_size",tolua_interface_Interface_hash_size00);
   tolua_function(tolua_S,"hash_printable",tolua_interface_Interface_hash_printable00);
   tolua_function(tolua_S,"is_writer",tolua_interface_Interface_is_writer00);
   tolua_function(tolua_S,"set_from_chunk",tolua_interface_Interface_set_from_chunk00);
   tolua_function(tolua_S,"create_message",tolua_interface_Interface_create_message00);
   tolua_function(tolua_S,"read",tolua_interface_Interface_read00);
   tolua_function(tolua_S,"write",tolua_interface_Interface_write00);
   tolua_function(tolua_S,"has_writer",tolua_interface_Interface_has_writer00);
   tolua_function(tolua_S,"num_readers",tolua_interface_Interface_num_readers00);
   tolua_function(tolua_S,"msgq_enqueue_copy",tolua_interface_Interface_msgq_enqueue_copy00);
   tolua_function(tolua_S,"msgq_remove",tolua_interface_Interface_msgq_remove00);
   tolua_function(tolua_S,"msgq_remove",tolua_interface_Interface_msgq_remove01);
   tolua_function(tolua_S,"msgq_size",tolua_interface_Interface_msgq_size00);
   tolua_function(tolua_S,"msgq_flush",tolua_interface_Interface_msgq_flush00);
   tolua_function(tolua_S,"msgq_lock",tolua_interface_Interface_msgq_lock00);
   tolua_function(tolua_S,"msgq_try_lock",tolua_interface_Interface_msgq_try_lock00);
   tolua_function(tolua_S,"msgq_unlock",tolua_interface_Interface_msgq_unlock00);
   tolua_function(tolua_S,"msgq_pop",tolua_interface_Interface_msgq_pop00);
   tolua_function(tolua_S,"msgq_first",tolua_interface_Interface_msgq_first00);
   tolua_function(tolua_S,"msgq_empty",tolua_interface_Interface_msgq_empty00);
  tolua_endmodule(tolua_S);
  #ifdef __cplusplus
  tolua_cclass(tolua_S,"MessageQueue","MessageQueue","",tolua_collect_MessageQueue);
  #else
  tolua_cclass(tolua_S,"MessageQueue","MessageQueue","",NULL);
  #endif
  tolua_beginmodule(tolua_S,"MessageQueue");
   tolua_function(tolua_S,"new",tolua_interface_MessageQueue_new00);
   tolua_function(tolua_S,"new_local",tolua_interface_MessageQueue_new00_local);
   tolua_function(tolua_S,".call",tolua_interface_MessageQueue_new00_local);
   tolua_function(tolua_S,"delete",tolua_interface_MessageQueue_delete00);
   #ifdef __cplusplus
   tolua_cclass(tolua_S,"MessageIterator","MessageQueue::MessageIterator","",tolua_collect_MessageQueue__MessageIterator);
   #else
   tolua_cclass(tolua_S,"MessageIterator","MessageQueue::MessageIterator","",NULL);
   #endif
   tolua_beginmodule(tolua_S,"MessageIterator");
    tolua_function(tolua_S,"new",tolua_interface_MessageQueue_MessageIterator_new00);
    tolua_function(tolua_S,"new_local",tolua_interface_MessageQueue_MessageIterator_new00_local);
    tolua_function(tolua_S,".call",tolua_interface_MessageQueue_MessageIterator_new00_local);
    tolua_function(tolua_S,"new",tolua_interface_MessageQueue_MessageIterator_new01);
    tolua_function(tolua_S,"new_local",tolua_interface_MessageQueue_MessageIterator_new01_local);
    tolua_function(tolua_S,".call",tolua_interface_MessageQueue_MessageIterator_new01_local);
    tolua_function(tolua_S,".add",tolua_interface_MessageQueue_MessageIterator__add00);
    tolua_function(tolua_S,".eq",tolua_interface_MessageQueue_MessageIterator__eq00);
    tolua_function(tolua_S,".mul",tolua_interface_MessageQueue_MessageIterator__mul00);
    tolua_function(tolua_S,"id",tolua_interface_MessageQueue_MessageIterator_id00);
   tolua_endmodule(tolua_S);
   tolua_function(tolua_S,"append",tolua_interface_MessageQueue_append00);
   tolua_function(tolua_S,"remove",tolua_interface_MessageQueue_remove00);
   tolua_function(tolua_S,"remove",tolua_interface_MessageQueue_remove01);
   tolua_function(tolua_S,"insert_after",tolua_interface_MessageQueue_insert_after00);
   tolua_function(tolua_S,"size",tolua_interface_MessageQueue_size00);
   tolua_function(tolua_S,"flush",tolua_interface_MessageQueue_flush00);
   tolua_function(tolua_S,"empty",tolua_interface_MessageQueue_empty00);
   tolua_function(tolua_S,"lock",tolua_interface_MessageQueue_lock00);
   tolua_function(tolua_S,"try_lock",tolua_interface_MessageQueue_try_lock00);
   tolua_function(tolua_S,"unlock",tolua_interface_MessageQueue_unlock00);
   tolua_function(tolua_S,"first",tolua_interface_MessageQueue_first00);
   tolua_function(tolua_S,"pop",tolua_interface_MessageQueue_pop00);
   tolua_function(tolua_S,"begin",tolua_interface_MessageQueue_begin00);
   tolua_function(tolua_S,"end",tolua_interface_MessageQueue_end00);
  tolua_endmodule(tolua_S);
  #ifdef __cplusplus
  tolua_cclass(tolua_S,"Message","Message","RefCount",tolua_collect_Message);
  #else
  tolua_cclass(tolua_S,"Message","Message","RefCount",NULL);
  #endif
  tolua_beginmodule(tolua_S,"Message");
   tolua_constant(tolua_S,"Undefined",Message::Undefined);
   tolua_constant(tolua_S,"Enqueued",Message::Enqueued);
   tolua_constant(tolua_S,"InProgress",Message::InProgress);
   tolua_constant(tolua_S,"Success",Message::Success);
   tolua_constant(tolua_S,"Failure",Message::Failure);
   tolua_function(tolua_S,"new",tolua_interface_Message_new00);
   tolua_function(tolua_S,"new_local",tolua_interface_Message_new00_local);
   tolua_function(tolua_S,".call",tolua_interface_Message_new00_local);
   tolua_function(tolua_S,"new",tolua_interface_Message_new01);
   tolua_function(tolua_S,"new_local",tolua_interface_Message_new01_local);
   tolua_function(tolua_S,".call",tolua_interface_Message_new01_local);
   tolua_function(tolua_S,"new",tolua_interface_Message_new02);
   tolua_function(tolua_S,"new_local",tolua_interface_Message_new02_local);
   tolua_function(tolua_S,".call",tolua_interface_Message_new02_local);
   tolua_function(tolua_S,"set_status",tolua_interface_Message_set_status00);
   tolua_function(tolua_S,"status",tolua_interface_Message_status00);
   tolua_function(tolua_S,"set_sub_status",tolua_interface_Message_set_sub_status00);
   tolua_function(tolua_S,"sub_status",tolua_interface_Message_sub_status00);
   tolua_function(tolua_S,"sender_id",tolua_interface_Message_sender_id00);
   tolua_function(tolua_S,"sender_thread_name",tolua_interface_Message_sender_thread_name00);
   tolua_function(tolua_S,"interface",tolua_interface_Message_interface00);
   tolua_function(tolua_S,"type",tolua_interface_Message_type00);
   tolua_function(tolua_S,"datachunk",tolua_interface_Message_datachunk00);
   tolua_function(tolua_S,"datasize",tolua_interface_Message_datasize00);
   tolua_function(tolua_S,"set_from_chunk",tolua_interface_Message_set_from_chunk00);
   tolua_function(tolua_S,"ref",tolua_interface_Message_ref00);
   tolua_function(tolua_S,"unref",tolua_interface_Message_unref00);
   tolua_function(tolua_S,"refcount",tolua_interface_Message_refcount00);
  tolua_endmodule(tolua_S);
 tolua_endmodule(tolua_S);
 return 1;
}


extern "C" {
#if defined(LUA_VERSION_NUM) && LUA_VERSION_NUM >= 501
 TOLUA_API int luaopen_interface (lua_State* tolua_S) {
 return tolua_interface_open(tolua_S);
};
#endif
}


