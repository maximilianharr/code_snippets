// Generated by the protocol buffer compiler.  DO NOT EDIT!
// source: loc_data.proto

#include "loc_data.pb.h"

#include <algorithm>

#include <google/protobuf/stubs/common.h>
#include <google/protobuf/stubs/port.h>
#include <google/protobuf/stubs/once.h>
#include <google/protobuf/io/coded_stream.h>
#include <google/protobuf/wire_format_lite_inl.h>
#include <google/protobuf/descriptor.h>
#include <google/protobuf/generated_message_reflection.h>
#include <google/protobuf/reflection_ops.h>
#include <google/protobuf/wire_format.h>
// This is a temporary google only hack
#ifdef GOOGLE_PROTOBUF_ENFORCE_UNIQUENESS
#include "third_party/protobuf/version.h"
#endif
// @@protoc_insertion_point(includes)
class LocDataDefaultTypeInternal {
 public:
  ::google::protobuf::internal::ExplicitlyConstructed<LocData>
      _instance;
} _LocData_default_instance_;
namespace protobuf_loc_5fdata_2eproto {
void InitDefaultsLocDataImpl() {
  GOOGLE_PROTOBUF_VERIFY_VERSION;

#ifdef GOOGLE_PROTOBUF_ENFORCE_UNIQUENESS
  ::google::protobuf::internal::InitProtobufDefaultsForceUnique();
#else
  ::google::protobuf::internal::InitProtobufDefaults();
#endif  // GOOGLE_PROTOBUF_ENFORCE_UNIQUENESS
  {
    void* ptr = &::_LocData_default_instance_;
    new (ptr) ::LocData();
    ::google::protobuf::internal::OnShutdownDestroyMessage(ptr);
  }
  ::LocData::InitAsDefaultInstance();
}

void InitDefaultsLocData() {
  static GOOGLE_PROTOBUF_DECLARE_ONCE(once);
  ::google::protobuf::GoogleOnceInit(&once, &InitDefaultsLocDataImpl);
}

::google::protobuf::Metadata file_level_metadata[1];

const ::google::protobuf::uint32 TableStruct::offsets[] GOOGLE_PROTOBUF_ATTRIBUTE_SECTION_VARIABLE(protodesc_cold) = {
  GOOGLE_PROTOBUF_GENERATED_MESSAGE_FIELD_OFFSET(::LocData, _has_bits_),
  GOOGLE_PROTOBUF_GENERATED_MESSAGE_FIELD_OFFSET(::LocData, _internal_metadata_),
  ~0u,  // no _extensions_
  ~0u,  // no _oneof_case_
  ~0u,  // no _weak_field_map_
  GOOGLE_PROTOBUF_GENERATED_MESSAGE_FIELD_OFFSET(::LocData, lat_),
  GOOGLE_PROTOBUF_GENERATED_MESSAGE_FIELD_OFFSET(::LocData, lon_),
  GOOGLE_PROTOBUF_GENERATED_MESSAGE_FIELD_OFFSET(::LocData, valid_),
  0,
  1,
  2,
};
static const ::google::protobuf::internal::MigrationSchema schemas[] GOOGLE_PROTOBUF_ATTRIBUTE_SECTION_VARIABLE(protodesc_cold) = {
  { 0, 8, sizeof(::LocData)},
};

static ::google::protobuf::Message const * const file_default_instances[] = {
  reinterpret_cast<const ::google::protobuf::Message*>(&::_LocData_default_instance_),
};

void protobuf_AssignDescriptors() {
  AddDescriptors();
  ::google::protobuf::MessageFactory* factory = NULL;
  AssignDescriptors(
      "loc_data.proto", schemas, file_default_instances, TableStruct::offsets, factory,
      file_level_metadata, NULL, NULL);
}

void protobuf_AssignDescriptorsOnce() {
  static GOOGLE_PROTOBUF_DECLARE_ONCE(once);
  ::google::protobuf::GoogleOnceInit(&once, &protobuf_AssignDescriptors);
}

void protobuf_RegisterTypes(const ::std::string&) GOOGLE_PROTOBUF_ATTRIBUTE_COLD;
void protobuf_RegisterTypes(const ::std::string&) {
  protobuf_AssignDescriptorsOnce();
  ::google::protobuf::internal::RegisterAllTypes(file_level_metadata, 1);
}

void AddDescriptorsImpl() {
  InitDefaults();
  static const char descriptor[] GOOGLE_PROTOBUF_ATTRIBUTE_SECTION_VARIABLE(protodesc_cold) = {
      "\n\016loc_data.proto\"2\n\007LocData\022\013\n\003lat\030\001 \002(\001"
      "\022\013\n\003lon\030\002 \002(\001\022\r\n\005valid\030\003 \001(\005"
  };
  ::google::protobuf::DescriptorPool::InternalAddGeneratedFile(
      descriptor, 68);
  ::google::protobuf::MessageFactory::InternalRegisterGeneratedFile(
    "loc_data.proto", &protobuf_RegisterTypes);
}

void AddDescriptors() {
  static GOOGLE_PROTOBUF_DECLARE_ONCE(once);
  ::google::protobuf::GoogleOnceInit(&once, &AddDescriptorsImpl);
}
// Force AddDescriptors() to be called at dynamic initialization time.
struct StaticDescriptorInitializer {
  StaticDescriptorInitializer() {
    AddDescriptors();
  }
} static_descriptor_initializer;
}  // namespace protobuf_loc_5fdata_2eproto

// ===================================================================

void LocData::InitAsDefaultInstance() {
}
#if !defined(_MSC_VER) || _MSC_VER >= 1900
const int LocData::kLatFieldNumber;
const int LocData::kLonFieldNumber;
const int LocData::kValidFieldNumber;
#endif  // !defined(_MSC_VER) || _MSC_VER >= 1900

LocData::LocData()
  : ::google::protobuf::Message(), _internal_metadata_(NULL) {
  if (GOOGLE_PREDICT_TRUE(this != internal_default_instance())) {
    ::protobuf_loc_5fdata_2eproto::InitDefaultsLocData();
  }
  SharedCtor();
  // @@protoc_insertion_point(constructor:LocData)
}
LocData::LocData(const LocData& from)
  : ::google::protobuf::Message(),
      _internal_metadata_(NULL),
      _has_bits_(from._has_bits_),
      _cached_size_(0) {
  _internal_metadata_.MergeFrom(from._internal_metadata_);
  ::memcpy(&lat_, &from.lat_,
    static_cast<size_t>(reinterpret_cast<char*>(&valid_) -
    reinterpret_cast<char*>(&lat_)) + sizeof(valid_));
  // @@protoc_insertion_point(copy_constructor:LocData)
}

void LocData::SharedCtor() {
  _cached_size_ = 0;
  ::memset(&lat_, 0, static_cast<size_t>(
      reinterpret_cast<char*>(&valid_) -
      reinterpret_cast<char*>(&lat_)) + sizeof(valid_));
}

LocData::~LocData() {
  // @@protoc_insertion_point(destructor:LocData)
  SharedDtor();
}

void LocData::SharedDtor() {
}

void LocData::SetCachedSize(int size) const {
  GOOGLE_SAFE_CONCURRENT_WRITES_BEGIN();
  _cached_size_ = size;
  GOOGLE_SAFE_CONCURRENT_WRITES_END();
}
const ::google::protobuf::Descriptor* LocData::descriptor() {
  ::protobuf_loc_5fdata_2eproto::protobuf_AssignDescriptorsOnce();
  return ::protobuf_loc_5fdata_2eproto::file_level_metadata[kIndexInFileMessages].descriptor;
}

const LocData& LocData::default_instance() {
  ::protobuf_loc_5fdata_2eproto::InitDefaultsLocData();
  return *internal_default_instance();
}

LocData* LocData::New(::google::protobuf::Arena* arena) const {
  LocData* n = new LocData;
  if (arena != NULL) {
    arena->Own(n);
  }
  return n;
}

void LocData::Clear() {
// @@protoc_insertion_point(message_clear_start:LocData)
  ::google::protobuf::uint32 cached_has_bits = 0;
  // Prevent compiler warnings about cached_has_bits being unused
  (void) cached_has_bits;

  cached_has_bits = _has_bits_[0];
  if (cached_has_bits & 7u) {
    ::memset(&lat_, 0, static_cast<size_t>(
        reinterpret_cast<char*>(&valid_) -
        reinterpret_cast<char*>(&lat_)) + sizeof(valid_));
  }
  _has_bits_.Clear();
  _internal_metadata_.Clear();
}

bool LocData::MergePartialFromCodedStream(
    ::google::protobuf::io::CodedInputStream* input) {
#define DO_(EXPRESSION) if (!GOOGLE_PREDICT_TRUE(EXPRESSION)) goto failure
  ::google::protobuf::uint32 tag;
  // @@protoc_insertion_point(parse_start:LocData)
  for (;;) {
    ::std::pair< ::google::protobuf::uint32, bool> p = input->ReadTagWithCutoffNoLastTag(127u);
    tag = p.first;
    if (!p.second) goto handle_unusual;
    switch (::google::protobuf::internal::WireFormatLite::GetTagFieldNumber(tag)) {
      // required double lat = 1;
      case 1: {
        if (static_cast< ::google::protobuf::uint8>(tag) ==
            static_cast< ::google::protobuf::uint8>(9u /* 9 & 0xFF */)) {
          set_has_lat();
          DO_((::google::protobuf::internal::WireFormatLite::ReadPrimitive<
                   double, ::google::protobuf::internal::WireFormatLite::TYPE_DOUBLE>(
                 input, &lat_)));
        } else {
          goto handle_unusual;
        }
        break;
      }

      // required double lon = 2;
      case 2: {
        if (static_cast< ::google::protobuf::uint8>(tag) ==
            static_cast< ::google::protobuf::uint8>(17u /* 17 & 0xFF */)) {
          set_has_lon();
          DO_((::google::protobuf::internal::WireFormatLite::ReadPrimitive<
                   double, ::google::protobuf::internal::WireFormatLite::TYPE_DOUBLE>(
                 input, &lon_)));
        } else {
          goto handle_unusual;
        }
        break;
      }

      // optional int32 valid = 3;
      case 3: {
        if (static_cast< ::google::protobuf::uint8>(tag) ==
            static_cast< ::google::protobuf::uint8>(24u /* 24 & 0xFF */)) {
          set_has_valid();
          DO_((::google::protobuf::internal::WireFormatLite::ReadPrimitive<
                   ::google::protobuf::int32, ::google::protobuf::internal::WireFormatLite::TYPE_INT32>(
                 input, &valid_)));
        } else {
          goto handle_unusual;
        }
        break;
      }

      default: {
      handle_unusual:
        if (tag == 0) {
          goto success;
        }
        DO_(::google::protobuf::internal::WireFormat::SkipField(
              input, tag, _internal_metadata_.mutable_unknown_fields()));
        break;
      }
    }
  }
success:
  // @@protoc_insertion_point(parse_success:LocData)
  return true;
failure:
  // @@protoc_insertion_point(parse_failure:LocData)
  return false;
#undef DO_
}

void LocData::SerializeWithCachedSizes(
    ::google::protobuf::io::CodedOutputStream* output) const {
  // @@protoc_insertion_point(serialize_start:LocData)
  ::google::protobuf::uint32 cached_has_bits = 0;
  (void) cached_has_bits;

  cached_has_bits = _has_bits_[0];
  // required double lat = 1;
  if (cached_has_bits & 0x00000001u) {
    ::google::protobuf::internal::WireFormatLite::WriteDouble(1, this->lat(), output);
  }

  // required double lon = 2;
  if (cached_has_bits & 0x00000002u) {
    ::google::protobuf::internal::WireFormatLite::WriteDouble(2, this->lon(), output);
  }

  // optional int32 valid = 3;
  if (cached_has_bits & 0x00000004u) {
    ::google::protobuf::internal::WireFormatLite::WriteInt32(3, this->valid(), output);
  }

  if (_internal_metadata_.have_unknown_fields()) {
    ::google::protobuf::internal::WireFormat::SerializeUnknownFields(
        _internal_metadata_.unknown_fields(), output);
  }
  // @@protoc_insertion_point(serialize_end:LocData)
}

::google::protobuf::uint8* LocData::InternalSerializeWithCachedSizesToArray(
    bool deterministic, ::google::protobuf::uint8* target) const {
  (void)deterministic; // Unused
  // @@protoc_insertion_point(serialize_to_array_start:LocData)
  ::google::protobuf::uint32 cached_has_bits = 0;
  (void) cached_has_bits;

  cached_has_bits = _has_bits_[0];
  // required double lat = 1;
  if (cached_has_bits & 0x00000001u) {
    target = ::google::protobuf::internal::WireFormatLite::WriteDoubleToArray(1, this->lat(), target);
  }

  // required double lon = 2;
  if (cached_has_bits & 0x00000002u) {
    target = ::google::protobuf::internal::WireFormatLite::WriteDoubleToArray(2, this->lon(), target);
  }

  // optional int32 valid = 3;
  if (cached_has_bits & 0x00000004u) {
    target = ::google::protobuf::internal::WireFormatLite::WriteInt32ToArray(3, this->valid(), target);
  }

  if (_internal_metadata_.have_unknown_fields()) {
    target = ::google::protobuf::internal::WireFormat::SerializeUnknownFieldsToArray(
        _internal_metadata_.unknown_fields(), target);
  }
  // @@protoc_insertion_point(serialize_to_array_end:LocData)
  return target;
}

size_t LocData::RequiredFieldsByteSizeFallback() const {
// @@protoc_insertion_point(required_fields_byte_size_fallback_start:LocData)
  size_t total_size = 0;

  if (has_lat()) {
    // required double lat = 1;
    total_size += 1 + 8;
  }

  if (has_lon()) {
    // required double lon = 2;
    total_size += 1 + 8;
  }

  return total_size;
}
size_t LocData::ByteSizeLong() const {
// @@protoc_insertion_point(message_byte_size_start:LocData)
  size_t total_size = 0;

  if (_internal_metadata_.have_unknown_fields()) {
    total_size +=
      ::google::protobuf::internal::WireFormat::ComputeUnknownFieldsSize(
        _internal_metadata_.unknown_fields());
  }
  if (((_has_bits_[0] & 0x00000003) ^ 0x00000003) == 0) {  // All required fields are present.
    // required double lat = 1;
    total_size += 1 + 8;

    // required double lon = 2;
    total_size += 1 + 8;

  } else {
    total_size += RequiredFieldsByteSizeFallback();
  }
  // optional int32 valid = 3;
  if (has_valid()) {
    total_size += 1 +
      ::google::protobuf::internal::WireFormatLite::Int32Size(
        this->valid());
  }

  int cached_size = ::google::protobuf::internal::ToCachedSize(total_size);
  GOOGLE_SAFE_CONCURRENT_WRITES_BEGIN();
  _cached_size_ = cached_size;
  GOOGLE_SAFE_CONCURRENT_WRITES_END();
  return total_size;
}

void LocData::MergeFrom(const ::google::protobuf::Message& from) {
// @@protoc_insertion_point(generalized_merge_from_start:LocData)
  GOOGLE_DCHECK_NE(&from, this);
  const LocData* source =
      ::google::protobuf::internal::DynamicCastToGenerated<const LocData>(
          &from);
  if (source == NULL) {
  // @@protoc_insertion_point(generalized_merge_from_cast_fail:LocData)
    ::google::protobuf::internal::ReflectionOps::Merge(from, this);
  } else {
  // @@protoc_insertion_point(generalized_merge_from_cast_success:LocData)
    MergeFrom(*source);
  }
}

void LocData::MergeFrom(const LocData& from) {
// @@protoc_insertion_point(class_specific_merge_from_start:LocData)
  GOOGLE_DCHECK_NE(&from, this);
  _internal_metadata_.MergeFrom(from._internal_metadata_);
  ::google::protobuf::uint32 cached_has_bits = 0;
  (void) cached_has_bits;

  cached_has_bits = from._has_bits_[0];
  if (cached_has_bits & 7u) {
    if (cached_has_bits & 0x00000001u) {
      lat_ = from.lat_;
    }
    if (cached_has_bits & 0x00000002u) {
      lon_ = from.lon_;
    }
    if (cached_has_bits & 0x00000004u) {
      valid_ = from.valid_;
    }
    _has_bits_[0] |= cached_has_bits;
  }
}

void LocData::CopyFrom(const ::google::protobuf::Message& from) {
// @@protoc_insertion_point(generalized_copy_from_start:LocData)
  if (&from == this) return;
  Clear();
  MergeFrom(from);
}

void LocData::CopyFrom(const LocData& from) {
// @@protoc_insertion_point(class_specific_copy_from_start:LocData)
  if (&from == this) return;
  Clear();
  MergeFrom(from);
}

bool LocData::IsInitialized() const {
  if ((_has_bits_[0] & 0x00000003) != 0x00000003) return false;
  return true;
}

void LocData::Swap(LocData* other) {
  if (other == this) return;
  InternalSwap(other);
}
void LocData::InternalSwap(LocData* other) {
  using std::swap;
  swap(lat_, other->lat_);
  swap(lon_, other->lon_);
  swap(valid_, other->valid_);
  swap(_has_bits_[0], other->_has_bits_[0]);
  _internal_metadata_.Swap(&other->_internal_metadata_);
  swap(_cached_size_, other->_cached_size_);
}

::google::protobuf::Metadata LocData::GetMetadata() const {
  protobuf_loc_5fdata_2eproto::protobuf_AssignDescriptorsOnce();
  return ::protobuf_loc_5fdata_2eproto::file_level_metadata[kIndexInFileMessages];
}


// @@protoc_insertion_point(namespace_scope)

// @@protoc_insertion_point(global_scope)