/* Generated by the protocol buffer compiler.  DO NOT EDIT! */
/* Generated from: opelmsg.proto */

#ifndef PROTOBUF_C_opelmsg_2eproto__INCLUDED
#define PROTOBUF_C_opelmsg_2eproto__INCLUDED

#include <protobuf-c/protobuf-c.h>

PROTOBUF_C__BEGIN_DECLS

#if PROTOBUF_C_VERSION_NUMBER < 1000000
# error This file was generated by a newer version of protoc-c which is incompatible with your libprotobuf-c headers. Please update your headers.
#elif 1002001 < PROTOBUF_C_MIN_COMPILER_VERSION
# error This file was generated by an older version of protoc-c which is incompatible with your libprotobuf-c headers. Please regenerate this file with a newer version of protoc-c.
#endif


typedef struct _OpelMsg OpelMsg;


/* --- enums --- */


/* --- messages --- */

struct  _OpelMsg
{
  ProtobufCMessage base;
  char *member;
  char *stream;
};
#define OPEL_MSG__INIT \
 { PROTOBUF_C_MESSAGE_INIT (&opel_msg__descriptor) \
    , NULL, NULL }


/* OpelMsg methods */
void   opel_msg__init
                     (OpelMsg         *message);
size_t opel_msg__get_packed_size
                     (const OpelMsg   *message);
size_t opel_msg__pack
                     (const OpelMsg   *message,
                      uint8_t             *out);
size_t opel_msg__pack_to_buffer
                     (const OpelMsg   *message,
                      ProtobufCBuffer     *buffer);
OpelMsg *
       opel_msg__unpack
                     (ProtobufCAllocator  *allocator,
                      size_t               len,
                      const uint8_t       *data);
void   opel_msg__free_unpacked
                     (OpelMsg *message,
                      ProtobufCAllocator *allocator);
/* --- per-message closures --- */

typedef void (*OpelMsg_Closure)
                 (const OpelMsg *message,
                  void *closure_data);

/* --- services --- */


/* --- descriptors --- */

extern const ProtobufCMessageDescriptor opel_msg__descriptor;

PROTOBUF_C__END_DECLS


#endif  /* PROTOBUF_C_opelmsg_2eproto__INCLUDED */
