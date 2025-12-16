// generated from rosidl_typesupport_c/resource/idl__type_support.cpp.em
// with input from limo_interfaces:action/Speak.idl
// generated code does not contain a copyright notice

#include "cstddef"
#include "rosidl_runtime_c/message_type_support_struct.h"
#include "limo_interfaces/action/detail/speak__struct.h"
#include "limo_interfaces/action/detail/speak__type_support.h"
#include "rosidl_typesupport_c/identifier.h"
#include "rosidl_typesupport_c/message_type_support_dispatch.h"
#include "rosidl_typesupport_c/type_support_map.h"
#include "rosidl_typesupport_c/visibility_control.h"
#include "rosidl_typesupport_interface/macros.h"

namespace limo_interfaces
{

namespace action
{

namespace rosidl_typesupport_c
{

typedef struct _Speak_Goal_type_support_ids_t
{
  const char * typesupport_identifier[2];
} _Speak_Goal_type_support_ids_t;

static const _Speak_Goal_type_support_ids_t _Speak_Goal_message_typesupport_ids = {
  {
    "rosidl_typesupport_fastrtps_c",  // ::rosidl_typesupport_fastrtps_c::typesupport_identifier,
    "rosidl_typesupport_introspection_c",  // ::rosidl_typesupport_introspection_c::typesupport_identifier,
  }
};

typedef struct _Speak_Goal_type_support_symbol_names_t
{
  const char * symbol_name[2];
} _Speak_Goal_type_support_symbol_names_t;

#define STRINGIFY_(s) #s
#define STRINGIFY(s) STRINGIFY_(s)

static const _Speak_Goal_type_support_symbol_names_t _Speak_Goal_message_typesupport_symbol_names = {
  {
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, limo_interfaces, action, Speak_Goal)),
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, limo_interfaces, action, Speak_Goal)),
  }
};

typedef struct _Speak_Goal_type_support_data_t
{
  void * data[2];
} _Speak_Goal_type_support_data_t;

static _Speak_Goal_type_support_data_t _Speak_Goal_message_typesupport_data = {
  {
    0,  // will store the shared library later
    0,  // will store the shared library later
  }
};

static const type_support_map_t _Speak_Goal_message_typesupport_map = {
  2,
  "limo_interfaces",
  &_Speak_Goal_message_typesupport_ids.typesupport_identifier[0],
  &_Speak_Goal_message_typesupport_symbol_names.symbol_name[0],
  &_Speak_Goal_message_typesupport_data.data[0],
};

static const rosidl_message_type_support_t Speak_Goal_message_type_support_handle = {
  rosidl_typesupport_c__typesupport_identifier,
  reinterpret_cast<const type_support_map_t *>(&_Speak_Goal_message_typesupport_map),
  rosidl_typesupport_c__get_message_typesupport_handle_function,
};

}  // namespace rosidl_typesupport_c

}  // namespace action

}  // namespace limo_interfaces

#ifdef __cplusplus
extern "C"
{
#endif

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_c, limo_interfaces, action, Speak_Goal)() {
  return &::limo_interfaces::action::rosidl_typesupport_c::Speak_Goal_message_type_support_handle;
}

#ifdef __cplusplus
}
#endif

// already included above
// #include "cstddef"
// already included above
// #include "rosidl_runtime_c/message_type_support_struct.h"
// already included above
// #include "limo_interfaces/action/detail/speak__struct.h"
// already included above
// #include "limo_interfaces/action/detail/speak__type_support.h"
// already included above
// #include "rosidl_typesupport_c/identifier.h"
// already included above
// #include "rosidl_typesupport_c/message_type_support_dispatch.h"
// already included above
// #include "rosidl_typesupport_c/type_support_map.h"
// already included above
// #include "rosidl_typesupport_c/visibility_control.h"
// already included above
// #include "rosidl_typesupport_interface/macros.h"

namespace limo_interfaces
{

namespace action
{

namespace rosidl_typesupport_c
{

typedef struct _Speak_Result_type_support_ids_t
{
  const char * typesupport_identifier[2];
} _Speak_Result_type_support_ids_t;

static const _Speak_Result_type_support_ids_t _Speak_Result_message_typesupport_ids = {
  {
    "rosidl_typesupport_fastrtps_c",  // ::rosidl_typesupport_fastrtps_c::typesupport_identifier,
    "rosidl_typesupport_introspection_c",  // ::rosidl_typesupport_introspection_c::typesupport_identifier,
  }
};

typedef struct _Speak_Result_type_support_symbol_names_t
{
  const char * symbol_name[2];
} _Speak_Result_type_support_symbol_names_t;

#define STRINGIFY_(s) #s
#define STRINGIFY(s) STRINGIFY_(s)

static const _Speak_Result_type_support_symbol_names_t _Speak_Result_message_typesupport_symbol_names = {
  {
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, limo_interfaces, action, Speak_Result)),
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, limo_interfaces, action, Speak_Result)),
  }
};

typedef struct _Speak_Result_type_support_data_t
{
  void * data[2];
} _Speak_Result_type_support_data_t;

static _Speak_Result_type_support_data_t _Speak_Result_message_typesupport_data = {
  {
    0,  // will store the shared library later
    0,  // will store the shared library later
  }
};

static const type_support_map_t _Speak_Result_message_typesupport_map = {
  2,
  "limo_interfaces",
  &_Speak_Result_message_typesupport_ids.typesupport_identifier[0],
  &_Speak_Result_message_typesupport_symbol_names.symbol_name[0],
  &_Speak_Result_message_typesupport_data.data[0],
};

static const rosidl_message_type_support_t Speak_Result_message_type_support_handle = {
  rosidl_typesupport_c__typesupport_identifier,
  reinterpret_cast<const type_support_map_t *>(&_Speak_Result_message_typesupport_map),
  rosidl_typesupport_c__get_message_typesupport_handle_function,
};

}  // namespace rosidl_typesupport_c

}  // namespace action

}  // namespace limo_interfaces

#ifdef __cplusplus
extern "C"
{
#endif

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_c, limo_interfaces, action, Speak_Result)() {
  return &::limo_interfaces::action::rosidl_typesupport_c::Speak_Result_message_type_support_handle;
}

#ifdef __cplusplus
}
#endif

// already included above
// #include "cstddef"
// already included above
// #include "rosidl_runtime_c/message_type_support_struct.h"
// already included above
// #include "limo_interfaces/action/detail/speak__struct.h"
// already included above
// #include "limo_interfaces/action/detail/speak__type_support.h"
// already included above
// #include "rosidl_typesupport_c/identifier.h"
// already included above
// #include "rosidl_typesupport_c/message_type_support_dispatch.h"
// already included above
// #include "rosidl_typesupport_c/type_support_map.h"
// already included above
// #include "rosidl_typesupport_c/visibility_control.h"
// already included above
// #include "rosidl_typesupport_interface/macros.h"

namespace limo_interfaces
{

namespace action
{

namespace rosidl_typesupport_c
{

typedef struct _Speak_Feedback_type_support_ids_t
{
  const char * typesupport_identifier[2];
} _Speak_Feedback_type_support_ids_t;

static const _Speak_Feedback_type_support_ids_t _Speak_Feedback_message_typesupport_ids = {
  {
    "rosidl_typesupport_fastrtps_c",  // ::rosidl_typesupport_fastrtps_c::typesupport_identifier,
    "rosidl_typesupport_introspection_c",  // ::rosidl_typesupport_introspection_c::typesupport_identifier,
  }
};

typedef struct _Speak_Feedback_type_support_symbol_names_t
{
  const char * symbol_name[2];
} _Speak_Feedback_type_support_symbol_names_t;

#define STRINGIFY_(s) #s
#define STRINGIFY(s) STRINGIFY_(s)

static const _Speak_Feedback_type_support_symbol_names_t _Speak_Feedback_message_typesupport_symbol_names = {
  {
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, limo_interfaces, action, Speak_Feedback)),
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, limo_interfaces, action, Speak_Feedback)),
  }
};

typedef struct _Speak_Feedback_type_support_data_t
{
  void * data[2];
} _Speak_Feedback_type_support_data_t;

static _Speak_Feedback_type_support_data_t _Speak_Feedback_message_typesupport_data = {
  {
    0,  // will store the shared library later
    0,  // will store the shared library later
  }
};

static const type_support_map_t _Speak_Feedback_message_typesupport_map = {
  2,
  "limo_interfaces",
  &_Speak_Feedback_message_typesupport_ids.typesupport_identifier[0],
  &_Speak_Feedback_message_typesupport_symbol_names.symbol_name[0],
  &_Speak_Feedback_message_typesupport_data.data[0],
};

static const rosidl_message_type_support_t Speak_Feedback_message_type_support_handle = {
  rosidl_typesupport_c__typesupport_identifier,
  reinterpret_cast<const type_support_map_t *>(&_Speak_Feedback_message_typesupport_map),
  rosidl_typesupport_c__get_message_typesupport_handle_function,
};

}  // namespace rosidl_typesupport_c

}  // namespace action

}  // namespace limo_interfaces

#ifdef __cplusplus
extern "C"
{
#endif

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_c, limo_interfaces, action, Speak_Feedback)() {
  return &::limo_interfaces::action::rosidl_typesupport_c::Speak_Feedback_message_type_support_handle;
}

#ifdef __cplusplus
}
#endif

// already included above
// #include "cstddef"
// already included above
// #include "rosidl_runtime_c/message_type_support_struct.h"
// already included above
// #include "limo_interfaces/action/detail/speak__struct.h"
// already included above
// #include "limo_interfaces/action/detail/speak__type_support.h"
// already included above
// #include "rosidl_typesupport_c/identifier.h"
// already included above
// #include "rosidl_typesupport_c/message_type_support_dispatch.h"
// already included above
// #include "rosidl_typesupport_c/type_support_map.h"
// already included above
// #include "rosidl_typesupport_c/visibility_control.h"
// already included above
// #include "rosidl_typesupport_interface/macros.h"

namespace limo_interfaces
{

namespace action
{

namespace rosidl_typesupport_c
{

typedef struct _Speak_SendGoal_Request_type_support_ids_t
{
  const char * typesupport_identifier[2];
} _Speak_SendGoal_Request_type_support_ids_t;

static const _Speak_SendGoal_Request_type_support_ids_t _Speak_SendGoal_Request_message_typesupport_ids = {
  {
    "rosidl_typesupport_fastrtps_c",  // ::rosidl_typesupport_fastrtps_c::typesupport_identifier,
    "rosidl_typesupport_introspection_c",  // ::rosidl_typesupport_introspection_c::typesupport_identifier,
  }
};

typedef struct _Speak_SendGoal_Request_type_support_symbol_names_t
{
  const char * symbol_name[2];
} _Speak_SendGoal_Request_type_support_symbol_names_t;

#define STRINGIFY_(s) #s
#define STRINGIFY(s) STRINGIFY_(s)

static const _Speak_SendGoal_Request_type_support_symbol_names_t _Speak_SendGoal_Request_message_typesupport_symbol_names = {
  {
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, limo_interfaces, action, Speak_SendGoal_Request)),
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, limo_interfaces, action, Speak_SendGoal_Request)),
  }
};

typedef struct _Speak_SendGoal_Request_type_support_data_t
{
  void * data[2];
} _Speak_SendGoal_Request_type_support_data_t;

static _Speak_SendGoal_Request_type_support_data_t _Speak_SendGoal_Request_message_typesupport_data = {
  {
    0,  // will store the shared library later
    0,  // will store the shared library later
  }
};

static const type_support_map_t _Speak_SendGoal_Request_message_typesupport_map = {
  2,
  "limo_interfaces",
  &_Speak_SendGoal_Request_message_typesupport_ids.typesupport_identifier[0],
  &_Speak_SendGoal_Request_message_typesupport_symbol_names.symbol_name[0],
  &_Speak_SendGoal_Request_message_typesupport_data.data[0],
};

static const rosidl_message_type_support_t Speak_SendGoal_Request_message_type_support_handle = {
  rosidl_typesupport_c__typesupport_identifier,
  reinterpret_cast<const type_support_map_t *>(&_Speak_SendGoal_Request_message_typesupport_map),
  rosidl_typesupport_c__get_message_typesupport_handle_function,
};

}  // namespace rosidl_typesupport_c

}  // namespace action

}  // namespace limo_interfaces

#ifdef __cplusplus
extern "C"
{
#endif

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_c, limo_interfaces, action, Speak_SendGoal_Request)() {
  return &::limo_interfaces::action::rosidl_typesupport_c::Speak_SendGoal_Request_message_type_support_handle;
}

#ifdef __cplusplus
}
#endif

// already included above
// #include "cstddef"
// already included above
// #include "rosidl_runtime_c/message_type_support_struct.h"
// already included above
// #include "limo_interfaces/action/detail/speak__struct.h"
// already included above
// #include "limo_interfaces/action/detail/speak__type_support.h"
// already included above
// #include "rosidl_typesupport_c/identifier.h"
// already included above
// #include "rosidl_typesupport_c/message_type_support_dispatch.h"
// already included above
// #include "rosidl_typesupport_c/type_support_map.h"
// already included above
// #include "rosidl_typesupport_c/visibility_control.h"
// already included above
// #include "rosidl_typesupport_interface/macros.h"

namespace limo_interfaces
{

namespace action
{

namespace rosidl_typesupport_c
{

typedef struct _Speak_SendGoal_Response_type_support_ids_t
{
  const char * typesupport_identifier[2];
} _Speak_SendGoal_Response_type_support_ids_t;

static const _Speak_SendGoal_Response_type_support_ids_t _Speak_SendGoal_Response_message_typesupport_ids = {
  {
    "rosidl_typesupport_fastrtps_c",  // ::rosidl_typesupport_fastrtps_c::typesupport_identifier,
    "rosidl_typesupport_introspection_c",  // ::rosidl_typesupport_introspection_c::typesupport_identifier,
  }
};

typedef struct _Speak_SendGoal_Response_type_support_symbol_names_t
{
  const char * symbol_name[2];
} _Speak_SendGoal_Response_type_support_symbol_names_t;

#define STRINGIFY_(s) #s
#define STRINGIFY(s) STRINGIFY_(s)

static const _Speak_SendGoal_Response_type_support_symbol_names_t _Speak_SendGoal_Response_message_typesupport_symbol_names = {
  {
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, limo_interfaces, action, Speak_SendGoal_Response)),
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, limo_interfaces, action, Speak_SendGoal_Response)),
  }
};

typedef struct _Speak_SendGoal_Response_type_support_data_t
{
  void * data[2];
} _Speak_SendGoal_Response_type_support_data_t;

static _Speak_SendGoal_Response_type_support_data_t _Speak_SendGoal_Response_message_typesupport_data = {
  {
    0,  // will store the shared library later
    0,  // will store the shared library later
  }
};

static const type_support_map_t _Speak_SendGoal_Response_message_typesupport_map = {
  2,
  "limo_interfaces",
  &_Speak_SendGoal_Response_message_typesupport_ids.typesupport_identifier[0],
  &_Speak_SendGoal_Response_message_typesupport_symbol_names.symbol_name[0],
  &_Speak_SendGoal_Response_message_typesupport_data.data[0],
};

static const rosidl_message_type_support_t Speak_SendGoal_Response_message_type_support_handle = {
  rosidl_typesupport_c__typesupport_identifier,
  reinterpret_cast<const type_support_map_t *>(&_Speak_SendGoal_Response_message_typesupport_map),
  rosidl_typesupport_c__get_message_typesupport_handle_function,
};

}  // namespace rosidl_typesupport_c

}  // namespace action

}  // namespace limo_interfaces

#ifdef __cplusplus
extern "C"
{
#endif

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_c, limo_interfaces, action, Speak_SendGoal_Response)() {
  return &::limo_interfaces::action::rosidl_typesupport_c::Speak_SendGoal_Response_message_type_support_handle;
}

#ifdef __cplusplus
}
#endif

// already included above
// #include "cstddef"
#include "rosidl_runtime_c/service_type_support_struct.h"
// already included above
// #include "limo_interfaces/action/detail/speak__type_support.h"
// already included above
// #include "rosidl_typesupport_c/identifier.h"
#include "rosidl_typesupport_c/service_type_support_dispatch.h"
// already included above
// #include "rosidl_typesupport_c/type_support_map.h"
// already included above
// #include "rosidl_typesupport_interface/macros.h"

namespace limo_interfaces
{

namespace action
{

namespace rosidl_typesupport_c
{

typedef struct _Speak_SendGoal_type_support_ids_t
{
  const char * typesupport_identifier[2];
} _Speak_SendGoal_type_support_ids_t;

static const _Speak_SendGoal_type_support_ids_t _Speak_SendGoal_service_typesupport_ids = {
  {
    "rosidl_typesupport_fastrtps_c",  // ::rosidl_typesupport_fastrtps_c::typesupport_identifier,
    "rosidl_typesupport_introspection_c",  // ::rosidl_typesupport_introspection_c::typesupport_identifier,
  }
};

typedef struct _Speak_SendGoal_type_support_symbol_names_t
{
  const char * symbol_name[2];
} _Speak_SendGoal_type_support_symbol_names_t;

#define STRINGIFY_(s) #s
#define STRINGIFY(s) STRINGIFY_(s)

static const _Speak_SendGoal_type_support_symbol_names_t _Speak_SendGoal_service_typesupport_symbol_names = {
  {
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, limo_interfaces, action, Speak_SendGoal)),
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(rosidl_typesupport_introspection_c, limo_interfaces, action, Speak_SendGoal)),
  }
};

typedef struct _Speak_SendGoal_type_support_data_t
{
  void * data[2];
} _Speak_SendGoal_type_support_data_t;

static _Speak_SendGoal_type_support_data_t _Speak_SendGoal_service_typesupport_data = {
  {
    0,  // will store the shared library later
    0,  // will store the shared library later
  }
};

static const type_support_map_t _Speak_SendGoal_service_typesupport_map = {
  2,
  "limo_interfaces",
  &_Speak_SendGoal_service_typesupport_ids.typesupport_identifier[0],
  &_Speak_SendGoal_service_typesupport_symbol_names.symbol_name[0],
  &_Speak_SendGoal_service_typesupport_data.data[0],
};

static const rosidl_service_type_support_t Speak_SendGoal_service_type_support_handle = {
  rosidl_typesupport_c__typesupport_identifier,
  reinterpret_cast<const type_support_map_t *>(&_Speak_SendGoal_service_typesupport_map),
  rosidl_typesupport_c__get_service_typesupport_handle_function,
};

}  // namespace rosidl_typesupport_c

}  // namespace action

}  // namespace limo_interfaces

#ifdef __cplusplus
extern "C"
{
#endif

const rosidl_service_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(rosidl_typesupport_c, limo_interfaces, action, Speak_SendGoal)() {
  return &::limo_interfaces::action::rosidl_typesupport_c::Speak_SendGoal_service_type_support_handle;
}

#ifdef __cplusplus
}
#endif

// already included above
// #include "cstddef"
// already included above
// #include "rosidl_runtime_c/message_type_support_struct.h"
// already included above
// #include "limo_interfaces/action/detail/speak__struct.h"
// already included above
// #include "limo_interfaces/action/detail/speak__type_support.h"
// already included above
// #include "rosidl_typesupport_c/identifier.h"
// already included above
// #include "rosidl_typesupport_c/message_type_support_dispatch.h"
// already included above
// #include "rosidl_typesupport_c/type_support_map.h"
// already included above
// #include "rosidl_typesupport_c/visibility_control.h"
// already included above
// #include "rosidl_typesupport_interface/macros.h"

namespace limo_interfaces
{

namespace action
{

namespace rosidl_typesupport_c
{

typedef struct _Speak_GetResult_Request_type_support_ids_t
{
  const char * typesupport_identifier[2];
} _Speak_GetResult_Request_type_support_ids_t;

static const _Speak_GetResult_Request_type_support_ids_t _Speak_GetResult_Request_message_typesupport_ids = {
  {
    "rosidl_typesupport_fastrtps_c",  // ::rosidl_typesupport_fastrtps_c::typesupport_identifier,
    "rosidl_typesupport_introspection_c",  // ::rosidl_typesupport_introspection_c::typesupport_identifier,
  }
};

typedef struct _Speak_GetResult_Request_type_support_symbol_names_t
{
  const char * symbol_name[2];
} _Speak_GetResult_Request_type_support_symbol_names_t;

#define STRINGIFY_(s) #s
#define STRINGIFY(s) STRINGIFY_(s)

static const _Speak_GetResult_Request_type_support_symbol_names_t _Speak_GetResult_Request_message_typesupport_symbol_names = {
  {
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, limo_interfaces, action, Speak_GetResult_Request)),
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, limo_interfaces, action, Speak_GetResult_Request)),
  }
};

typedef struct _Speak_GetResult_Request_type_support_data_t
{
  void * data[2];
} _Speak_GetResult_Request_type_support_data_t;

static _Speak_GetResult_Request_type_support_data_t _Speak_GetResult_Request_message_typesupport_data = {
  {
    0,  // will store the shared library later
    0,  // will store the shared library later
  }
};

static const type_support_map_t _Speak_GetResult_Request_message_typesupport_map = {
  2,
  "limo_interfaces",
  &_Speak_GetResult_Request_message_typesupport_ids.typesupport_identifier[0],
  &_Speak_GetResult_Request_message_typesupport_symbol_names.symbol_name[0],
  &_Speak_GetResult_Request_message_typesupport_data.data[0],
};

static const rosidl_message_type_support_t Speak_GetResult_Request_message_type_support_handle = {
  rosidl_typesupport_c__typesupport_identifier,
  reinterpret_cast<const type_support_map_t *>(&_Speak_GetResult_Request_message_typesupport_map),
  rosidl_typesupport_c__get_message_typesupport_handle_function,
};

}  // namespace rosidl_typesupport_c

}  // namespace action

}  // namespace limo_interfaces

#ifdef __cplusplus
extern "C"
{
#endif

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_c, limo_interfaces, action, Speak_GetResult_Request)() {
  return &::limo_interfaces::action::rosidl_typesupport_c::Speak_GetResult_Request_message_type_support_handle;
}

#ifdef __cplusplus
}
#endif

// already included above
// #include "cstddef"
// already included above
// #include "rosidl_runtime_c/message_type_support_struct.h"
// already included above
// #include "limo_interfaces/action/detail/speak__struct.h"
// already included above
// #include "limo_interfaces/action/detail/speak__type_support.h"
// already included above
// #include "rosidl_typesupport_c/identifier.h"
// already included above
// #include "rosidl_typesupport_c/message_type_support_dispatch.h"
// already included above
// #include "rosidl_typesupport_c/type_support_map.h"
// already included above
// #include "rosidl_typesupport_c/visibility_control.h"
// already included above
// #include "rosidl_typesupport_interface/macros.h"

namespace limo_interfaces
{

namespace action
{

namespace rosidl_typesupport_c
{

typedef struct _Speak_GetResult_Response_type_support_ids_t
{
  const char * typesupport_identifier[2];
} _Speak_GetResult_Response_type_support_ids_t;

static const _Speak_GetResult_Response_type_support_ids_t _Speak_GetResult_Response_message_typesupport_ids = {
  {
    "rosidl_typesupport_fastrtps_c",  // ::rosidl_typesupport_fastrtps_c::typesupport_identifier,
    "rosidl_typesupport_introspection_c",  // ::rosidl_typesupport_introspection_c::typesupport_identifier,
  }
};

typedef struct _Speak_GetResult_Response_type_support_symbol_names_t
{
  const char * symbol_name[2];
} _Speak_GetResult_Response_type_support_symbol_names_t;

#define STRINGIFY_(s) #s
#define STRINGIFY(s) STRINGIFY_(s)

static const _Speak_GetResult_Response_type_support_symbol_names_t _Speak_GetResult_Response_message_typesupport_symbol_names = {
  {
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, limo_interfaces, action, Speak_GetResult_Response)),
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, limo_interfaces, action, Speak_GetResult_Response)),
  }
};

typedef struct _Speak_GetResult_Response_type_support_data_t
{
  void * data[2];
} _Speak_GetResult_Response_type_support_data_t;

static _Speak_GetResult_Response_type_support_data_t _Speak_GetResult_Response_message_typesupport_data = {
  {
    0,  // will store the shared library later
    0,  // will store the shared library later
  }
};

static const type_support_map_t _Speak_GetResult_Response_message_typesupport_map = {
  2,
  "limo_interfaces",
  &_Speak_GetResult_Response_message_typesupport_ids.typesupport_identifier[0],
  &_Speak_GetResult_Response_message_typesupport_symbol_names.symbol_name[0],
  &_Speak_GetResult_Response_message_typesupport_data.data[0],
};

static const rosidl_message_type_support_t Speak_GetResult_Response_message_type_support_handle = {
  rosidl_typesupport_c__typesupport_identifier,
  reinterpret_cast<const type_support_map_t *>(&_Speak_GetResult_Response_message_typesupport_map),
  rosidl_typesupport_c__get_message_typesupport_handle_function,
};

}  // namespace rosidl_typesupport_c

}  // namespace action

}  // namespace limo_interfaces

#ifdef __cplusplus
extern "C"
{
#endif

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_c, limo_interfaces, action, Speak_GetResult_Response)() {
  return &::limo_interfaces::action::rosidl_typesupport_c::Speak_GetResult_Response_message_type_support_handle;
}

#ifdef __cplusplus
}
#endif

// already included above
// #include "cstddef"
// already included above
// #include "rosidl_runtime_c/service_type_support_struct.h"
// already included above
// #include "limo_interfaces/action/detail/speak__type_support.h"
// already included above
// #include "rosidl_typesupport_c/identifier.h"
// already included above
// #include "rosidl_typesupport_c/service_type_support_dispatch.h"
// already included above
// #include "rosidl_typesupport_c/type_support_map.h"
// already included above
// #include "rosidl_typesupport_interface/macros.h"

namespace limo_interfaces
{

namespace action
{

namespace rosidl_typesupport_c
{

typedef struct _Speak_GetResult_type_support_ids_t
{
  const char * typesupport_identifier[2];
} _Speak_GetResult_type_support_ids_t;

static const _Speak_GetResult_type_support_ids_t _Speak_GetResult_service_typesupport_ids = {
  {
    "rosidl_typesupport_fastrtps_c",  // ::rosidl_typesupport_fastrtps_c::typesupport_identifier,
    "rosidl_typesupport_introspection_c",  // ::rosidl_typesupport_introspection_c::typesupport_identifier,
  }
};

typedef struct _Speak_GetResult_type_support_symbol_names_t
{
  const char * symbol_name[2];
} _Speak_GetResult_type_support_symbol_names_t;

#define STRINGIFY_(s) #s
#define STRINGIFY(s) STRINGIFY_(s)

static const _Speak_GetResult_type_support_symbol_names_t _Speak_GetResult_service_typesupport_symbol_names = {
  {
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, limo_interfaces, action, Speak_GetResult)),
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(rosidl_typesupport_introspection_c, limo_interfaces, action, Speak_GetResult)),
  }
};

typedef struct _Speak_GetResult_type_support_data_t
{
  void * data[2];
} _Speak_GetResult_type_support_data_t;

static _Speak_GetResult_type_support_data_t _Speak_GetResult_service_typesupport_data = {
  {
    0,  // will store the shared library later
    0,  // will store the shared library later
  }
};

static const type_support_map_t _Speak_GetResult_service_typesupport_map = {
  2,
  "limo_interfaces",
  &_Speak_GetResult_service_typesupport_ids.typesupport_identifier[0],
  &_Speak_GetResult_service_typesupport_symbol_names.symbol_name[0],
  &_Speak_GetResult_service_typesupport_data.data[0],
};

static const rosidl_service_type_support_t Speak_GetResult_service_type_support_handle = {
  rosidl_typesupport_c__typesupport_identifier,
  reinterpret_cast<const type_support_map_t *>(&_Speak_GetResult_service_typesupport_map),
  rosidl_typesupport_c__get_service_typesupport_handle_function,
};

}  // namespace rosidl_typesupport_c

}  // namespace action

}  // namespace limo_interfaces

#ifdef __cplusplus
extern "C"
{
#endif

const rosidl_service_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(rosidl_typesupport_c, limo_interfaces, action, Speak_GetResult)() {
  return &::limo_interfaces::action::rosidl_typesupport_c::Speak_GetResult_service_type_support_handle;
}

#ifdef __cplusplus
}
#endif

// already included above
// #include "cstddef"
// already included above
// #include "rosidl_runtime_c/message_type_support_struct.h"
// already included above
// #include "limo_interfaces/action/detail/speak__struct.h"
// already included above
// #include "limo_interfaces/action/detail/speak__type_support.h"
// already included above
// #include "rosidl_typesupport_c/identifier.h"
// already included above
// #include "rosidl_typesupport_c/message_type_support_dispatch.h"
// already included above
// #include "rosidl_typesupport_c/type_support_map.h"
// already included above
// #include "rosidl_typesupport_c/visibility_control.h"
// already included above
// #include "rosidl_typesupport_interface/macros.h"

namespace limo_interfaces
{

namespace action
{

namespace rosidl_typesupport_c
{

typedef struct _Speak_FeedbackMessage_type_support_ids_t
{
  const char * typesupport_identifier[2];
} _Speak_FeedbackMessage_type_support_ids_t;

static const _Speak_FeedbackMessage_type_support_ids_t _Speak_FeedbackMessage_message_typesupport_ids = {
  {
    "rosidl_typesupport_fastrtps_c",  // ::rosidl_typesupport_fastrtps_c::typesupport_identifier,
    "rosidl_typesupport_introspection_c",  // ::rosidl_typesupport_introspection_c::typesupport_identifier,
  }
};

typedef struct _Speak_FeedbackMessage_type_support_symbol_names_t
{
  const char * symbol_name[2];
} _Speak_FeedbackMessage_type_support_symbol_names_t;

#define STRINGIFY_(s) #s
#define STRINGIFY(s) STRINGIFY_(s)

static const _Speak_FeedbackMessage_type_support_symbol_names_t _Speak_FeedbackMessage_message_typesupport_symbol_names = {
  {
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, limo_interfaces, action, Speak_FeedbackMessage)),
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, limo_interfaces, action, Speak_FeedbackMessage)),
  }
};

typedef struct _Speak_FeedbackMessage_type_support_data_t
{
  void * data[2];
} _Speak_FeedbackMessage_type_support_data_t;

static _Speak_FeedbackMessage_type_support_data_t _Speak_FeedbackMessage_message_typesupport_data = {
  {
    0,  // will store the shared library later
    0,  // will store the shared library later
  }
};

static const type_support_map_t _Speak_FeedbackMessage_message_typesupport_map = {
  2,
  "limo_interfaces",
  &_Speak_FeedbackMessage_message_typesupport_ids.typesupport_identifier[0],
  &_Speak_FeedbackMessage_message_typesupport_symbol_names.symbol_name[0],
  &_Speak_FeedbackMessage_message_typesupport_data.data[0],
};

static const rosidl_message_type_support_t Speak_FeedbackMessage_message_type_support_handle = {
  rosidl_typesupport_c__typesupport_identifier,
  reinterpret_cast<const type_support_map_t *>(&_Speak_FeedbackMessage_message_typesupport_map),
  rosidl_typesupport_c__get_message_typesupport_handle_function,
};

}  // namespace rosidl_typesupport_c

}  // namespace action

}  // namespace limo_interfaces

#ifdef __cplusplus
extern "C"
{
#endif

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_c, limo_interfaces, action, Speak_FeedbackMessage)() {
  return &::limo_interfaces::action::rosidl_typesupport_c::Speak_FeedbackMessage_message_type_support_handle;
}

#ifdef __cplusplus
}
#endif

#include "action_msgs/msg/goal_status_array.h"
#include "action_msgs/srv/cancel_goal.h"
#include "limo_interfaces/action/speak.h"
// already included above
// #include "limo_interfaces/action/detail/speak__type_support.h"

static rosidl_action_type_support_t _limo_interfaces__action__Speak__typesupport_c;

#ifdef __cplusplus
extern "C"
{
#endif

const rosidl_action_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__ACTION_SYMBOL_NAME(
  rosidl_typesupport_c, limo_interfaces, action, Speak)()
{
  // Thread-safe by always writing the same values to the static struct
  _limo_interfaces__action__Speak__typesupport_c.goal_service_type_support =
    ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(
    rosidl_typesupport_c, limo_interfaces, action, Speak_SendGoal)();
  _limo_interfaces__action__Speak__typesupport_c.result_service_type_support =
    ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(
    rosidl_typesupport_c, limo_interfaces, action, Speak_GetResult)();
  _limo_interfaces__action__Speak__typesupport_c.cancel_service_type_support =
    ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(
    rosidl_typesupport_c, action_msgs, srv, CancelGoal)();
  _limo_interfaces__action__Speak__typesupport_c.feedback_message_type_support =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(
    rosidl_typesupport_c, limo_interfaces, action, Speak_FeedbackMessage)();
  _limo_interfaces__action__Speak__typesupport_c.status_message_type_support =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(
    rosidl_typesupport_c, action_msgs, msg, GoalStatusArray)();

  return &_limo_interfaces__action__Speak__typesupport_c;
}

#ifdef __cplusplus
}
#endif
