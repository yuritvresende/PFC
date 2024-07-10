// generated from rosidl_typesupport_fastrtps_c/resource/idl__type_support_c.cpp.em
// with input from robotiq_85_msgs:msg/GripperCmd.idl
// generated code does not contain a copyright notice
#include "robotiq_85_msgs/msg/detail/gripper_cmd__rosidl_typesupport_fastrtps_c.h"


#include <cassert>
#include <limits>
#include <string>
#include "rosidl_typesupport_fastrtps_c/identifier.h"
#include "rosidl_typesupport_fastrtps_c/wstring_conversion.hpp"
#include "rosidl_typesupport_fastrtps_cpp/message_type_support.h"
#include "robotiq_85_msgs/msg/rosidl_typesupport_fastrtps_c__visibility_control.h"
#include "robotiq_85_msgs/msg/detail/gripper_cmd__struct.h"
#include "robotiq_85_msgs/msg/detail/gripper_cmd__functions.h"
#include "fastcdr/Cdr.h"

#ifndef _WIN32
# pragma GCC diagnostic push
# pragma GCC diagnostic ignored "-Wunused-parameter"
# ifdef __clang__
#  pragma clang diagnostic ignored "-Wdeprecated-register"
#  pragma clang diagnostic ignored "-Wreturn-type-c-linkage"
# endif
#endif
#ifndef _WIN32
# pragma GCC diagnostic pop
#endif

// includes and forward declarations of message dependencies and their conversion functions

#if defined(__cplusplus)
extern "C"
{
#endif


// forward declare type support functions


using _GripperCmd__ros_msg_type = robotiq_85_msgs__msg__GripperCmd;

static bool _GripperCmd__cdr_serialize(
  const void * untyped_ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  const _GripperCmd__ros_msg_type * ros_message = static_cast<const _GripperCmd__ros_msg_type *>(untyped_ros_message);
  // Field name: emergency_release
  {
    cdr << (ros_message->emergency_release ? true : false);
  }

  // Field name: emergency_release_dir
  {
    cdr << ros_message->emergency_release_dir;
  }

  // Field name: stop
  {
    cdr << (ros_message->stop ? true : false);
  }

  // Field name: position
  {
    cdr << ros_message->position;
  }

  // Field name: speed
  {
    cdr << ros_message->speed;
  }

  // Field name: force
  {
    cdr << ros_message->force;
  }

  return true;
}

static bool _GripperCmd__cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  void * untyped_ros_message)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  _GripperCmd__ros_msg_type * ros_message = static_cast<_GripperCmd__ros_msg_type *>(untyped_ros_message);
  // Field name: emergency_release
  {
    uint8_t tmp;
    cdr >> tmp;
    ros_message->emergency_release = tmp ? true : false;
  }

  // Field name: emergency_release_dir
  {
    cdr >> ros_message->emergency_release_dir;
  }

  // Field name: stop
  {
    uint8_t tmp;
    cdr >> tmp;
    ros_message->stop = tmp ? true : false;
  }

  // Field name: position
  {
    cdr >> ros_message->position;
  }

  // Field name: speed
  {
    cdr >> ros_message->speed;
  }

  // Field name: force
  {
    cdr >> ros_message->force;
  }

  return true;
}  // NOLINT(readability/fn_size)

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_robotiq_85_msgs
size_t get_serialized_size_robotiq_85_msgs__msg__GripperCmd(
  const void * untyped_ros_message,
  size_t current_alignment)
{
  const _GripperCmd__ros_msg_type * ros_message = static_cast<const _GripperCmd__ros_msg_type *>(untyped_ros_message);
  (void)ros_message;
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;

  // field.name emergency_release
  {
    size_t item_size = sizeof(ros_message->emergency_release);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name emergency_release_dir
  {
    size_t item_size = sizeof(ros_message->emergency_release_dir);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name stop
  {
    size_t item_size = sizeof(ros_message->stop);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name position
  {
    size_t item_size = sizeof(ros_message->position);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name speed
  {
    size_t item_size = sizeof(ros_message->speed);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name force
  {
    size_t item_size = sizeof(ros_message->force);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  return current_alignment - initial_alignment;
}

static uint32_t _GripperCmd__get_serialized_size(const void * untyped_ros_message)
{
  return static_cast<uint32_t>(
    get_serialized_size_robotiq_85_msgs__msg__GripperCmd(
      untyped_ros_message, 0));
}

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_robotiq_85_msgs
size_t max_serialized_size_robotiq_85_msgs__msg__GripperCmd(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment)
{
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  size_t last_member_size = 0;
  (void)last_member_size;
  (void)padding;
  (void)wchar_size;

  full_bounded = true;
  is_plain = true;

  // member: emergency_release
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }
  // member: emergency_release_dir
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }
  // member: stop
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }
  // member: position
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }
  // member: speed
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }
  // member: force
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  size_t ret_val = current_alignment - initial_alignment;
  if (is_plain) {
    // All members are plain, and type is not empty.
    // We still need to check that the in-memory alignment
    // is the same as the CDR mandated alignment.
    using DataType = robotiq_85_msgs__msg__GripperCmd;
    is_plain =
      (
      offsetof(DataType, force) +
      last_member_size
      ) == ret_val;
  }

  return ret_val;
}

static size_t _GripperCmd__max_serialized_size(char & bounds_info)
{
  bool full_bounded;
  bool is_plain;
  size_t ret_val;

  ret_val = max_serialized_size_robotiq_85_msgs__msg__GripperCmd(
    full_bounded, is_plain, 0);

  bounds_info =
    is_plain ? ROSIDL_TYPESUPPORT_FASTRTPS_PLAIN_TYPE :
    full_bounded ? ROSIDL_TYPESUPPORT_FASTRTPS_BOUNDED_TYPE : ROSIDL_TYPESUPPORT_FASTRTPS_UNBOUNDED_TYPE;
  return ret_val;
}


static message_type_support_callbacks_t __callbacks_GripperCmd = {
  "robotiq_85_msgs::msg",
  "GripperCmd",
  _GripperCmd__cdr_serialize,
  _GripperCmd__cdr_deserialize,
  _GripperCmd__get_serialized_size,
  _GripperCmd__max_serialized_size
};

static rosidl_message_type_support_t _GripperCmd__type_support = {
  rosidl_typesupport_fastrtps_c__identifier,
  &__callbacks_GripperCmd,
  get_message_typesupport_handle_function,
};

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, robotiq_85_msgs, msg, GripperCmd)() {
  return &_GripperCmd__type_support;
}

#if defined(__cplusplus)
}
#endif
