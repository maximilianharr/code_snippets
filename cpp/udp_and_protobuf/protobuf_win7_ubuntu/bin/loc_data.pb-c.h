/* Generated by the protocol buffer compiler.  DO NOT EDIT! */
/* Generated from: loc_data.proto */

#ifndef PROTOBUF_C_loc_5fdata_2eproto__INCLUDED
#define PROTOBUF_C_loc_5fdata_2eproto__INCLUDED

#include <protobuf-c/protobuf-c.h>

PROTOBUF_C__BEGIN_DECLS

#if PROTOBUF_C_VERSION_NUMBER < 1000000
# error This file was generated by a newer version of protoc-c which is incompatible with your libprotobuf-c headers. Please update your headers.
#elif 1002001 < PROTOBUF_C_MIN_COMPILER_VERSION
# error This file was generated by an older version of protoc-c which is incompatible with your libprotobuf-c headers. Please regenerate this file with a newer version of protoc-c.
#endif


typedef struct _LocData LocData;


/* --- enums --- */


/* --- messages --- */

struct  _LocData
{
  ProtobufCMessage base;
  double lat;
  double lon;
  protobuf_c_boolean has_valid;
  int32_t valid;
};
#define LOC_DATA__INIT \
 { PROTOBUF_C_MESSAGE_INIT (&loc_data__descriptor) \
    , 0, 0, 0,0 }


/* LocData methods */
void   loc_data__init
                     (LocData         *message);
size_t loc_data__get_packed_size
                     (const LocData   *message);
size_t loc_data__pack
                     (const LocData   *message,
                      uint8_t             *out);
size_t loc_data__pack_to_buffer
                     (const LocData   *message,
                      ProtobufCBuffer     *buffer);
LocData *
       loc_data__unpack
                     (ProtobufCAllocator  *allocator,
                      size_t               len,
                      const uint8_t       *data);
void   loc_data__free_unpacked
                     (LocData *message,
                      ProtobufCAllocator *allocator);
/* --- per-message closures --- */

typedef void (*LocData_Closure)
                 (const LocData *message,
                  void *closure_data);

/* --- services --- */


/* --- descriptors --- */

extern const ProtobufCMessageDescriptor loc_data__descriptor;

PROTOBUF_C__END_DECLS


#endif  /* PROTOBUF_C_loc_5fdata_2eproto__INCLUDED */
