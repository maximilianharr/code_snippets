/* Generated by the protocol buffer compiler.  DO NOT EDIT! */
/* Generated from: opelmsg.proto */

/* Do not generate deprecated warnings for self */
#ifndef PROTOBUF_C__NO_DEPRECATED
#define PROTOBUF_C__NO_DEPRECATED
#endif

#include "opelmsg.pb-c.h"
void   opel_msg__init
                     (OpelMsg         *message)
{
  static OpelMsg init_value = OPEL_MSG__INIT;
  *message = init_value;
}
size_t opel_msg__get_packed_size
                     (const OpelMsg *message)
{
  assert(message->base.descriptor == &opel_msg__descriptor);
  return protobuf_c_message_get_packed_size ((const ProtobufCMessage*)(message));
}
size_t opel_msg__pack
                     (const OpelMsg *message,
                      uint8_t       *out)
{
  assert(message->base.descriptor == &opel_msg__descriptor);
  return protobuf_c_message_pack ((const ProtobufCMessage*)message, out);
}
size_t opel_msg__pack_to_buffer
                     (const OpelMsg *message,
                      ProtobufCBuffer *buffer)
{
  assert(message->base.descriptor == &opel_msg__descriptor);
  return protobuf_c_message_pack_to_buffer ((const ProtobufCMessage*)message, buffer);
}
OpelMsg *
       opel_msg__unpack
                     (ProtobufCAllocator  *allocator,
                      size_t               len,
                      const uint8_t       *data)
{
  return (OpelMsg *)
     protobuf_c_message_unpack (&opel_msg__descriptor,
                                allocator, len, data);
}
void   opel_msg__free_unpacked
                     (OpelMsg *message,
                      ProtobufCAllocator *allocator)
{
  assert(message->base.descriptor == &opel_msg__descriptor);
  protobuf_c_message_free_unpacked ((ProtobufCMessage*)message, allocator);
}
static const ProtobufCFieldDescriptor opel_msg__field_descriptors[2] =
{
  {
    "member",
    1,
    PROTOBUF_C_LABEL_OPTIONAL,
    PROTOBUF_C_TYPE_STRING,
    0,   /* quantifier_offset */
    offsetof(OpelMsg, member),
    NULL,
    NULL,
    0,             /* flags */
    0,NULL,NULL    /* reserved1,reserved2, etc */
  },
  {
    "stream",
    2,
    PROTOBUF_C_LABEL_REQUIRED,
    PROTOBUF_C_TYPE_STRING,
    0,   /* quantifier_offset */
    offsetof(OpelMsg, stream),
    NULL,
    NULL,
    0,             /* flags */
    0,NULL,NULL    /* reserved1,reserved2, etc */
  },
};
static const unsigned opel_msg__field_indices_by_name[] = {
  0,   /* field[0] = member */
  1,   /* field[1] = stream */
};
static const ProtobufCIntRange opel_msg__number_ranges[1 + 1] =
{
  { 1, 0 },
  { 0, 2 }
};
const ProtobufCMessageDescriptor opel_msg__descriptor =
{
  PROTOBUF_C__MESSAGE_DESCRIPTOR_MAGIC,
  "opelMsg",
  "OpelMsg",
  "OpelMsg",
  "",
  sizeof(OpelMsg),
  2,
  opel_msg__field_descriptors,
  opel_msg__field_indices_by_name,
  1,  opel_msg__number_ranges,
  (ProtobufCMessageInit) opel_msg__init,
  NULL,NULL,NULL    /* reserved[123] */
};
