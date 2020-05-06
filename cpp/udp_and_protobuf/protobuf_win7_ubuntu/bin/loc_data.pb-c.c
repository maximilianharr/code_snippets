/* Generated by the protocol buffer compiler.  DO NOT EDIT! */
/* Generated from: loc_data.proto */

/* Do not generate deprecated warnings for self */
#ifndef PROTOBUF_C__NO_DEPRECATED
#define PROTOBUF_C__NO_DEPRECATED
#endif

#include "loc_data.pb-c.h"
void   loc_data__init
                     (LocData         *message)
{
  static LocData init_value = LOC_DATA__INIT;
  *message = init_value;
}
size_t loc_data__get_packed_size
                     (const LocData *message)
{
  assert(message->base.descriptor == &loc_data__descriptor);
  return protobuf_c_message_get_packed_size ((const ProtobufCMessage*)(message));
}
size_t loc_data__pack
                     (const LocData *message,
                      uint8_t       *out)
{
  assert(message->base.descriptor == &loc_data__descriptor);
  return protobuf_c_message_pack ((const ProtobufCMessage*)message, out);
}
size_t loc_data__pack_to_buffer
                     (const LocData *message,
                      ProtobufCBuffer *buffer)
{
  assert(message->base.descriptor == &loc_data__descriptor);
  return protobuf_c_message_pack_to_buffer ((const ProtobufCMessage*)message, buffer);
}
LocData *
       loc_data__unpack
                     (ProtobufCAllocator  *allocator,
                      size_t               len,
                      const uint8_t       *data)
{
  return (LocData *)
     protobuf_c_message_unpack (&loc_data__descriptor,
                                allocator, len, data);
}
void   loc_data__free_unpacked
                     (LocData *message,
                      ProtobufCAllocator *allocator)
{
  assert(message->base.descriptor == &loc_data__descriptor);
  protobuf_c_message_free_unpacked ((ProtobufCMessage*)message, allocator);
}
static const ProtobufCFieldDescriptor loc_data__field_descriptors[3] =
{
  {
    "lat",
    1,
    PROTOBUF_C_LABEL_REQUIRED,
    PROTOBUF_C_TYPE_DOUBLE,
    0,   /* quantifier_offset */
    offsetof(LocData, lat),
    NULL,
    NULL,
    0,             /* flags */
    0,NULL,NULL    /* reserved1,reserved2, etc */
  },
  {
    "lon",
    2,
    PROTOBUF_C_LABEL_REQUIRED,
    PROTOBUF_C_TYPE_DOUBLE,
    0,   /* quantifier_offset */
    offsetof(LocData, lon),
    NULL,
    NULL,
    0,             /* flags */
    0,NULL,NULL    /* reserved1,reserved2, etc */
  },
  {
    "valid",
    3,
    PROTOBUF_C_LABEL_OPTIONAL,
    PROTOBUF_C_TYPE_INT32,
    offsetof(LocData, has_valid),
    offsetof(LocData, valid),
    NULL,
    NULL,
    0,             /* flags */
    0,NULL,NULL    /* reserved1,reserved2, etc */
  },
};
static const unsigned loc_data__field_indices_by_name[] = {
  0,   /* field[0] = lat */
  1,   /* field[1] = lon */
  2,   /* field[2] = valid */
};
static const ProtobufCIntRange loc_data__number_ranges[1 + 1] =
{
  { 1, 0 },
  { 0, 3 }
};
const ProtobufCMessageDescriptor loc_data__descriptor =
{
  PROTOBUF_C__MESSAGE_DESCRIPTOR_MAGIC,
  "LocData",
  "LocData",
  "LocData",
  "",
  sizeof(LocData),
  3,
  loc_data__field_descriptors,
  loc_data__field_indices_by_name,
  1,  loc_data__number_ranges,
  (ProtobufCMessageInit) loc_data__init,
  NULL,NULL,NULL    /* reserved[123] */
};