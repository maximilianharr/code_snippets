/* Generated by the protocol buffer compiler.  DO NOT EDIT! */
/* Generated from: loc.proto */

#ifndef PROTOBUF_C_loc_2eproto__INCLUDED
#define PROTOBUF_C_loc_2eproto__INCLUDED

#include <protobuf-c/protobuf-c.h>

PROTOBUF_C__BEGIN_DECLS

#if PROTOBUF_C_VERSION_NUMBER < 1000000
# error This file was generated by a newer version of protoc-c which is incompatible with your libprotobuf-c headers. Please update your headers.
#elif 1002001 < PROTOBUF_C_MIN_COMPILER_VERSION
# error This file was generated by an older version of protoc-c which is incompatible with your libprotobuf-c headers. Please regenerate this file with a newer version of protoc-c.
#endif


typedef struct _Loc Loc;


/* --- enums --- */


/* --- messages --- */

struct  _Loc
{
  ProtobufCMessage base;
  double a;
  float b;
  int32_t c;
  int64_t d;
  uint32_t e;
  uint64_t f;
  int32_t g;
  int64_t h;
  uint32_t i;
  uint64_t j;
  protobuf_c_boolean k;
  char *l;
};
#define LOC__INIT \
 { PROTOBUF_C_MESSAGE_INIT (&loc__descriptor) \
    , 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, NULL }


/* Loc methods */
void   loc__init
                     (Loc         *message);
size_t loc__get_packed_size
                     (const Loc   *message);
size_t loc__pack
                     (const Loc   *message,
                      uint8_t             *out);
size_t loc__pack_to_buffer
                     (const Loc   *message,
                      ProtobufCBuffer     *buffer);
Loc *
       loc__unpack
                     (ProtobufCAllocator  *allocator,
                      size_t               len,
                      const uint8_t       *data);
void   loc__free_unpacked
                     (Loc *message,
                      ProtobufCAllocator *allocator);
/* --- per-message closures --- */

typedef void (*Loc_Closure)
                 (const Loc *message,
                  void *closure_data);

/* --- services --- */


/* --- descriptors --- */

extern const ProtobufCMessageDescriptor loc__descriptor;

PROTOBUF_C__END_DECLS


#endif  /* PROTOBUF_C_loc_2eproto__INCLUDED */
